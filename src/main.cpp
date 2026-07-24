/**
 * T-A7670G (A7670E-FASE built-in GNSS) GPS car tracker with onboard web UI.
 *   TRIP mode  (external/USB power present): stay awake, report every reportSec.
 *   PARK mode  (running on 18650, power lost): report once, then deep-sleep parkMin.
 * Power state is inferred from the battery ADC (GPIO35): on USB the reading is
 * pinned high; on battery it reads the real (lower, sagging) cell voltage.
 *   - Status page  (/)        live fix, sats, map+trail, power/mode, battery mV
 *   - Config page  (/config)  WiFi, Traccar, park interval, power threshold
 *   - AP fallback   TTGO-GPS-Setup ; mDNS http://ttgo-gps.local
 * Uplink is WiFi for now (4G to be added when SIM+antenna arrive).
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <HTTPUpdate.h>
#include "esp_sleep.h"
#include <math.h>

#define FW_VERSION "1.0.4"

// Build-time defaults injected from a gitignored .env (see load_env.py). Blank if unset —
// creds then come from the /config page (stored in NVS) and survive OTA.
#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASS
#define WIFI_PASS ""
#endif

// ---- board / modem pins ----
#define BOARD_POWERON_PIN 12
#define MODEM_RESET_PIN   5
#define MODEM_RESET_LEVEL HIGH
#define BOARD_PWRKEY_PIN  4
#define MODEM_DTR_PIN     25
#define MODEM_TX_PIN      26
#define MODEM_RX_PIN      27
#define BOARD_BAT_ADC_PIN 35     // battery voltage divider (V1.2/R2)

HardwareSerial SerialAT(1);
WebServer      server(80);
Preferences    prefs;

// Position polled from the modem via AT+CGNSSINFO (no NMEA streaming).
struct GnssFix {
    bool     valid = false;
    double   lat = 0, lon = 0;
    float    alt = 0, speedKn = 0, course = 0, hdop = 0;
    int      sats = 0;
    int      Y = 0, Mo = 0, D = 0, h = 0, m = 0, s = 0;
    uint32_t ageMs = 0;      // millis() at last valid update
} fix;

// ---- persisted config ----
struct Config {
    String   wifiSsid, wifiPass;
    String   traccarHost, deviceId;
    uint16_t traccarPort;
    uint16_t reportSec;      // TRIP-mode report interval (s)
    uint16_t parkMin;        // PARK-mode report/sleep interval (min)
    uint16_t powerThreshMv;  // battery mV at/above which we call it "external power"
    bool     traccarEnabled;
    bool     deepSleep;      // PARK: deep-sleep between reports (off = stay awake, reachable)
    // cellular (4G)
    String   apn, apnUser, apnPass, simPin;
    bool     cellEnabled;    // use 4G when WiFi is unavailable
    bool     preferCell;     // prefer 4G even when WiFi STA is connected
    // OTA
    String   otaRepo, otaAsset;   // GitHub owner/repo + asset filename
} cfg;
String otaStatus = "idle";

void loadConfig()
{
    prefs.begin("tracker", true);
    cfg.wifiSsid       = prefs.getString("ssid",  WIFI_SSID);
    cfg.wifiPass       = prefs.getString("pass",  WIFI_PASS);
    cfg.traccarHost    = prefs.getString("thost", "demo.traccar.org");
    cfg.traccarPort    = prefs.getUShort("tport", 5055);
    cfg.deviceId       = prefs.getString("did",   "ttgo-a7670-01");
    cfg.reportSec      = prefs.getUShort("rsec",  10);
    cfg.parkMin        = prefs.getUShort("pmin",  30);
    cfg.powerThreshMv  = prefs.getUShort("pth",   4150);
    cfg.traccarEnabled = prefs.getBool("ten",     true);
    cfg.deepSleep      = prefs.getBool("dsleep",  false);         // default OFF for now
    cfg.apn            = prefs.getString("apn",   "mobile.sky");  // Sky Mobile (O2); TM=ThingsMobile, iot.1nce.net=1NCE
    cfg.apnUser        = prefs.getString("apnu",  "");
    cfg.apnPass        = prefs.getString("apnp",  "");
    cfg.simPin         = prefs.getString("pin",   "");     // blank = no PIN
    cfg.cellEnabled    = prefs.getBool("cell",    true);
    cfg.preferCell     = prefs.getBool("pcell",   false);
    cfg.otaRepo        = prefs.getString("orepo", "");             // e.g. "callum/ttgo-tracker"
    cfg.otaAsset       = prefs.getString("oasset","firmware.bin");
    prefs.end();
}
void saveConfig()
{
    prefs.begin("tracker", false);
    prefs.putString("ssid",  cfg.wifiSsid);
    prefs.putString("pass",  cfg.wifiPass);
    prefs.putString("thost", cfg.traccarHost);
    prefs.putUShort("tport", cfg.traccarPort);
    prefs.putString("did",   cfg.deviceId);
    prefs.putUShort("rsec",  cfg.reportSec);
    prefs.putUShort("pmin",  cfg.parkMin);
    prefs.putUShort("pth",   cfg.powerThreshMv);
    prefs.putBool("ten",     cfg.traccarEnabled);
    prefs.putBool("dsleep",  cfg.deepSleep);
    prefs.putString("apn",   cfg.apn);
    prefs.putString("apnu",  cfg.apnUser);
    prefs.putString("apnp",  cfg.apnPass);
    prefs.putString("pin",   cfg.simPin);
    prefs.putBool("cell",    cfg.cellEnabled);
    prefs.putBool("pcell",   cfg.preferCell);
    prefs.putString("orepo", cfg.otaRepo);
    prefs.putString("oasset",cfg.otaAsset);
    prefs.end();
}

// ---- runtime state ----
bool      apMode = false;
int       lastPostCode = 0;
uint32_t  lastPostMs = 0;
uint32_t  battMv = 0;
bool      powerPresent = true;
const char *modeStr = "BOOT";
// cellular status (refreshed by pollModemStatus)
String    simStatus = "?";
int       cellRssiDbm = 0;     // 0 = unknown
bool      cellRegistered = false;
String    cellOperator = "";
// upload health + signal history
String    lastVia = "-";
uint32_t  lastOkMs = 0;
RTC_DATA_ATTR uint32_t rtcPostOk = 0;     // survive deep-sleep so park cycles accumulate
RTC_DATA_ATTR uint32_t rtcPostFail = 0;
#define SIG_MAX 48                        // ~16 min at one sample / 20s
int8_t    sigHist[SIG_MAX];
int       sigN = 0, sigHead = 0;
void pushSig(int dbm){ sigHist[sigHead] = (int8_t)dbm; sigHead = (sigHead + 1) % SIG_MAX; if (sigN < SIG_MAX) sigN++; }
void recordPost(int code, const char *via){
    lastPostCode = code; lastPostMs = millis(); lastVia = via;
    if (code == 200) { lastOkMs = millis(); rtcPostOk++; } else rtcPostFail++;
}
struct Pt { float lat, lon; };
#define TRACK_MAX 60
Pt   track[TRACK_MAX];
int  trackN = 0, trackHead = 0;
void pushTrack(float la, float lo) {
    track[trackHead] = {la, lo};
    trackHead = (trackHead + 1) % TRACK_MAX;
    if (trackN < TRACK_MAX) trackN++;
}

String atCmd(const char *cmd, uint32_t wait)
{
    while (SerialAT.available()) SerialAT.read();
    SerialAT.print(cmd); SerialAT.print("\r\n");
    String r; uint32_t end = millis() + wait;
    while (millis() < end) {
        while (SerialAT.available()) r += (char)SerialAT.read();
        if (r.indexOf("\r\nOK\r\n") >= 0 || r.indexOf("\r\nERROR") >= 0 || r.indexOf("+CME ERROR") >= 0) break;
    }
    r.trim(); return r;
}
long toEpoch(int Y,int M,int D,int h,int m,int s){
    static const int c[]={0,31,59,90,120,151,181,212,243,273,304,334};
    long days=(Y-1970)*365L+(Y-1969)/4; days+=c[M-1]+(D-1);
    if(M>2&&(Y%4==0&&(Y%100!=0||Y%400==0)))days++;
    return days*86400L+h*3600L+m*60L+s;
}
// Parse a "+CGNSSINFO: mode,gpsSV,gloSV,bdSV,lat,N/S,lon,E/W,ddmmyy,hhmmss.s,alt,spdKn,course,PDOP,HDOP,VDOP"
bool parseCGNSSINFO(const String &resp)
{
    int i = resp.indexOf("+CGNSSINFO:");
    if (i < 0) return false;
    String s = resp.substring(i + 11);
    int e = s.indexOf('\n'); if (e >= 0) s = s.substring(0, e);
    s.trim();
    String f[16]; int n = 0, start = 0;
    for (int k = 0; k <= (int)s.length() && n < 16; k++)
        if (k == (int)s.length() || s[k] == ',') { f[n++] = s.substring(start, k); start = k + 1; }
    if (n < 8 || f[0].length() == 0 || f[4].length() == 0) return false;   // no fix
    double latRaw = f[4].toDouble(), lonRaw = f[6].toDouble();
    double la = (int)(latRaw / 100) + fmod(latRaw, 100.0) / 60.0;
    double lo = (int)(lonRaw / 100) + fmod(lonRaw, 100.0) / 60.0;
    if (f[5] == "S") la = -la;
    if (f[7] == "W") lo = -lo;
    fix.lat = la; fix.lon = lo;
    fix.sats = f[1].toInt() + f[2].toInt() + f[3].toInt();
    if (f[8].length() >= 6) { fix.D = f[8].substring(0,2).toInt(); fix.Mo = f[8].substring(2,4).toInt(); fix.Y = 2000 + f[8].substring(4,6).toInt(); }
    if (f[9].length() >= 6) { fix.h = f[9].substring(0,2).toInt(); fix.m = f[9].substring(2,4).toInt(); fix.s = f[9].substring(4,6).toInt(); }
    if (n > 10) fix.alt     = f[10].toFloat();
    if (n > 11) fix.speedKn = f[11].toFloat();
    if (n > 12) fix.course  = f[12].toFloat();
    if (n > 14) fix.hdop    = f[14].toFloat();
    fix.valid = true; fix.ageMs = millis();
    return true;
}
void pollGnss()
{
    String r = atCmd("AT+CGNSSINFO", 1500);
    if (r.indexOf("+CGNSSINFO:") < 0) return;      // no reply this cycle; existing fix ages out
    if (!parseCGNSSINFO(r)) fix.valid = false;     // got a reply but no fix
}
bool haveFix(){ return fix.valid && (millis() - fix.ageMs < 8000); }

// forward declarations (plain .cpp has no auto-prototyping)
String buildTraccarUrl();
String reportCellular();
void   reportWiFi();
void   report();

// battery/power sensing: median-ish of a few samples, x2 for the divider
uint32_t readBatteryMv()
{
    uint32_t sum = 0; int n = 0;
    for (int i = 0; i < 16; i++) { uint32_t v = analogReadMilliVolts(BOARD_BAT_ADC_PIN); if (v) { sum += v; n++; } delay(2); }
    return n ? (sum / n) * 2 : 0;
}
#define NO_BATTERY_MV 2500      // below this = no cell fitted -> must be on USB (bench/dev)
void updatePower()
{
    battMv = readBatteryMv();
    // Two "external power present" cases:
    //  1) reading near 0  -> no battery installed, so we're running on USB
    //  2) reading >= threshold -> charger is holding the rail up (car ignition on)
    // Everything between = a real cell discharging on its own -> parked.
    powerPresent = (battMv < NO_BATTERY_MV) || (battMv >= cfg.powerThreshMv);
}

// extract a substring token after `key` up to the next CR/LF or comma set
static String afterKey(const String &s, const char *key)
{
    int i = s.indexOf(key); if (i < 0) return "";
    i += strlen(key); int e = i;
    while (e < (int)s.length() && s[e] != '\r' && s[e] != '\n') e++;
    String r = s.substring(i, e); r.trim(); return r;
}
// Query SIM + signal + registration. Shares the UART with the NMEA stream, so
// responses arrive amid $Gx sentences — we substring-match the tokens we want.
void pollModemStatus()
{
    String r = atCmd("AT+CPIN?", 600);
    if (r.indexOf("READY") >= 0)                              simStatus = "ready";
    else if (r.indexOf("SIM PIN") >= 0)                       simStatus = "PIN required";
    else if (r.indexOf("NOT INSERTED") >= 0 ||
             r.indexOf("ERROR") >= 0)                         simStatus = "no SIM";
    else if (r.indexOf("NOT READY") >= 0)                     simStatus = "not ready";
    else if (r.length() == 0)                                 simStatus = "no response";
    else                                                      simStatus = "unknown";

    r = atCmd("AT+CSQ", 600);
    String csq = afterKey(r, "+CSQ:");
    int comma = csq.indexOf(',');
    int val = (comma > 0) ? csq.substring(0, comma).toInt() : 99;
    cellRssiDbm = (val == 99 || val == 0) ? 0 : (-113 + 2 * val);

    r = atCmd("AT+CREG?", 600);
    String creg = afterKey(r, "+CREG:");
    // format: <n>,<stat>  -> stat 1=home, 5=roaming registered
    int c2 = creg.indexOf(',');
    int stat = (c2 >= 0) ? creg.substring(c2 + 1).toInt() : 0;
    cellRegistered = (stat == 1 || stat == 5);

    r = atCmd("AT+COPS?", 800);
    int q1 = r.indexOf('"'); int q2 = (q1 >= 0) ? r.indexOf('"', q1 + 1) : -1;
    cellOperator = (q1 >= 0 && q2 > q1) ? r.substring(q1 + 1, q2) : "";

    pushSig(cellRssiDbm ? cellRssiDbm : -113);   // log signal (unknown -> floor)
}

// ================= web pages (status + config) =================
const char PAGE_STATUS[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1"><title>TTGO GPS</title>
<link rel=stylesheet href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<style>
:root{color-scheme:dark light}
body{margin:0;font:15px/1.4 system-ui,sans-serif;background:#0e1116;color:#e6edf3}
header{display:flex;justify-content:space-between;align-items:center;padding:12px 16px;background:#161b22;border-bottom:1px solid #30363d}
h1{font-size:16px;margin:0}
a.btn{color:#58a6ff;text-decoration:none;border:1px solid #30363d;padding:6px 10px;border-radius:6px}
#map{height:40vh;width:100%;background:#222}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:10px;padding:16px}
.card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:10px 12px}
.card .k{font-size:11px;text-transform:uppercase;letter-spacing:.05em;color:#8b949e}
.card .v{font-size:20px;font-weight:600;margin-top:3px;word-break:break-all}
.fix{color:#3fb950}.nofix{color:#f85149}.trip{color:#3fb950}.park{color:#d29922}
.foot{padding:0 16px 20px;color:#8b949e;font-size:12px}
</style></head><body>
<header><h1>🛰️ TTGO GPS Tracker</h1><a class=btn href="/config">⚙ Config</a></header>
<div id=map></div><div class=grid id=grid></div>
<div style="padding:0 16px 8px"><div style="font-size:11px;text-transform:uppercase;letter-spacing:.05em;color:#8b949e;margin-bottom:6px">4G signal — last ~16 min</div><div id=sig></div></div>
<div class=foot id=foot></div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
let map,marker,line;
function initMap(){map=L.map('map').setView([0,0],2);
 L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19,attribution:'© OSM'}).addTo(map);}
function fmt(n,d){return n==null?'–':Number(n).toFixed(d)}
async function tick(){
 try{
  const s=await (await fetch('/api/status')).json();
  const cards=[
   ['Mode', s.power?'<span class=trip>TRIP</span>':'<span class=park>PARK</span>'],
   ['Power', s.power?'external ✓':'battery'],
   ['Battery', s.battmv+' mV'],
   ['Fix',s.fix?'<span class=fix>3D FIX</span>':'<span class=nofix>NO FIX</span>'],
   ['Latitude',s.fix?fmt(s.lat,6):'–'],['Longitude',s.fix?fmt(s.lon,6):'–'],
   ['Satellites',s.sats],['HDOP',fmt(s.hdop,1)],
   ['Altitude',s.fix?fmt(s.alt,0)+' m':'–'],['Speed',s.fix?fmt(s.kmh,1)+' km/h':'–'],
   ['UTC',s.utc||'–'],['WiFi',s.wifi+' ('+s.rssi+' dBm)'],['IP',s.ip],
   ['SIM',s.sim],
   ['Signal',s.celldbm?s.celldbm+' dBm':'–'],
   ['Network',s.cellreg?('<span class=fix>'+(s.cellop||'registered')+'</span>'):'not registered'],
   ['Traccar',s.ten?(s.postCode==200?'<span class=fix>OK 200</span>':'HTTP '+s.postCode):'off'],
   ['Last upload', s.okAgo>=0 ? ('<span class=fix>'+s.okAgo+'s ago</span> · '+s.via) : '<span class=nofix>none yet</span>'],
   ['Last attempt', s.postAgo>=0 ? ('HTTP '+s.postCode+' · '+s.postAgo+'s ago') : '–'],
   ['Uploads', '<span class=fix>'+s.okCount+'</span> ok / <span class=nofix>'+s.failCount+'</span> fail'],
   ['Firmware','v'+s.fw+(s.ota&&s.ota!='idle'?' · '+s.ota:'')],
   ['Uptime',s.up+' s']
  ];
  grid.innerHTML=cards.map(c=>`<div class=card><div class=k>${c[0]}</div><div class=v>${c[1]}</div></div>`).join('');
  foot.textContent='Device: '+s.did+'  →  '+s.thost+':'+s.tport+'   ·   trip '+s.rsec+'s / park '+s.pmin+'min   ·   deep-sleep '+(s.dsleep?'on':'off')+'   ·   4G '+(s.cell?'on':'off')+(s.pcell?' (preferred)':'')+' APN '+s.apn;
  if(s.fix){const p=[s.lat,s.lon];
   if(!marker){marker=L.marker(p).addTo(map);map.setView(p,16);}else marker.setLatLng(p);
   const t=await (await fetch('/api/track')).json();
   if(line)line.remove(); if(t.length>1)line=L.polyline(t,{color:'#58a6ff'}).addTo(map);}
 }catch(e){}
}
async function drawSig(){
 try{
  const a=await (await fetch('/api/sig')).json();
  if(!a.length){sig.innerHTML='<span style="color:#8b949e;font-size:12px">no samples yet</span>';return;}
  const w=Math.max(240,a.length*8),h=50,lo=-113,hi=-50;
  const pts=a.map((v,i)=>{const x=i*(w/(a.length>1?a.length-1:1));const c=Math.max(lo,Math.min(hi,v));const y=h-((c-lo)/(hi-lo))*h;return x.toFixed(0)+','+y.toFixed(1)}).join(' ');
  const cur=a[a.length-1];
  sig.innerHTML='<svg width="100%" height="50" viewBox="0 0 '+w+' '+h+'" preserveAspectRatio="none" style="background:#161b22;border:1px solid #30363d;border-radius:6px"><polyline fill="none" stroke="#58a6ff" stroke-width="2" points="'+pts+'"/></svg>'
   +'<div style="font-size:12px;color:#8b949e;margin-top:4px">now <b>'+cur+'</b> dBm · min '+Math.min(...a)+' · max '+Math.max(...a)+'  <span style=color:#6e7681>(−50 strong … −113 none)</span></div>';
 }catch(e){}
}
initMap();tick();drawSig();setInterval(()=>{tick();drawSig();},2000);
</script></body></html>
)HTML";

const char PAGE_CONFIG[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1"><title>Config</title>
<style>
body{margin:0;font:15px/1.5 system-ui,sans-serif;background:#0e1116;color:#e6edf3}
.wrap{max-width:460px;margin:0 auto;padding:20px}
h1{font-size:17px}h2{font-size:14px;color:#8b949e;margin-top:24px}
label{display:block;margin:14px 0 4px;font-size:13px;color:#8b949e}
input[type=text],input[type=password],input[type=number]{width:100%;box-sizing:border-box;padding:9px;border:1px solid #30363d;border-radius:6px;background:#0d1117;color:#e6edf3;font-size:15px}
.row{display:flex;align-items:center;gap:8px;margin-top:14px}
button{margin-top:22px;width:100%;padding:11px;border:0;border-radius:6px;background:#238636;color:#fff;font-size:15px;font-weight:600}
a{color:#58a6ff}.hint{font-size:12px;color:#6e7681;margin-top:3px}
</style></head><body><div class=wrap>
<p><a href="/">← Status</a></p><h1>⚙ Configuration</h1>
<form method=POST action="/save">
<h2>NETWORK</h2>
<label>WiFi SSID</label><input type=text name=ssid value="%SSID%">
<label>WiFi Password</label><input type=password name=pass value="%PASS%">
<h2>TRACCAR</h2>
<div class=row><input type=checkbox name=ten %TEN% id=ten><label for=ten style=margin:0>Upload to Traccar</label></div>
<label>Host</label><input type=text name=thost value="%THOST%">
<label>Port</label><input type=number name=tport value="%TPORT%">
<label>Device ID</label><input type=text name=did value="%DID%">
<h2>CELLULAR (4G)</h2>
<div class=row><input type=checkbox name=cell %CELL% id=cell><label for=cell style=margin:0>Use 4G when WiFi unavailable</label></div>
<div class=row><input type=checkbox name=pcell %PCELL% id=pcell><label for=pcell style=margin:0>Prefer 4G even when WiFi is connected</label></div>
<label>APN</label><input type=text name=apn value="%APN%">
<div class=hint>Sky Mobile = <b>mobile.sky</b> · Things Mobile = <b>TM</b> · 1NCE = <b>iot.1nce.net</b></div>
<label>APN username (usually blank)</label><input type=text name=apnu value="%APNU%">
<label>APN password (usually blank)</label><input type=text name=apnp value="%APNP%">
<label>SIM PIN (blank if none)</label><input type=text name=pin value="%PIN%">
<h2>REPORTING &amp; POWER</h2>
<label>Trip interval (seconds, on power)</label><input type=number name=rsec value="%RSEC%" min=5>
<label>Park interval (minutes, on battery)</label><input type=number name=pmin value="%PMIN%" min=1>
<label>Power-detect threshold (mV)</label><input type=number name=pth value="%PTH%" min=3000 max=5000>
<div class=hint>Battery reads above this = "external power" (TRIP). Set between the on-USB and on-battery readings shown on the status page.</div>
<div class=row><input type=checkbox name=dsleep %DSLEEP% id=dsleep><label for=dsleep style=margin:0>Deep-sleep when on battery (PARK)</label></div>
<div class=hint>Off = stay awake on battery (hotspot reachable, catches ignition within 5 s, uses more power). On = sleep between park reports to save battery.</div>
<h2>FIRMWARE (OTA)</h2>
<div class=hint>Running <b>v%FWVER%</b> · pulls the latest GitHub release over WiFi</div>
<label>GitHub repo (owner/repo)</label><input type=text name=orepo value="%OREPO%">
<label>Release asset filename</label><input type=text name=oasset value="%OASSET%">
<button type=submit>Save &amp; Reboot</button>
</form>
<a href="/ota" onclick="return confirm('Download the latest firmware from GitHub and flash it? The device will reboot.')" style="display:block;text-align:center;margin-top:14px;padding:11px;border:1px solid #d29922;border-radius:6px;color:#d29922;text-decoration:none;font-weight:600">&#8593; Update firmware now</a>
<div class=hint style="text-align:center">Save the repo first, then update. WiFi only.</div>
</div></body></html>
)HTML";

void handleStatus() {
    String j = "{";
    j += "\"fix\":" + String(haveFix() ? "true" : "false");
    if (haveFix()) {
        j += ",\"lat\":" + String(fix.lat, 6);
        j += ",\"lon\":" + String(fix.lon, 6);
        j += ",\"alt\":" + String(fix.alt, 1);
        j += ",\"kmh\":" + String(fix.speedKn * 1.852, 1);
    }
    j += ",\"sats\":" + String(fix.sats);
    j += ",\"hdop\":" + String(fix.hdop, 1);
    char utc[16] = "";
    if (haveFix()) snprintf(utc, sizeof(utc), "%02d:%02d:%02d", fix.h, fix.m, fix.s);
    j += ",\"utc\":\"" + String(utc) + "\"";
    j += ",\"battmv\":" + String(battMv);
    j += ",\"power\":" + String(powerPresent ? "true" : "false");
    j += ",\"sim\":\"" + simStatus + "\"";
    j += ",\"celldbm\":" + String(cellRssiDbm);
    j += ",\"cellreg\":" + String(cellRegistered ? "true" : "false");
    j += ",\"cellop\":\"" + cellOperator + "\"";
    j += ",\"apn\":\"" + cfg.apn + "\",\"cell\":" + String(cfg.cellEnabled ? "true" : "false");
    j += ",\"wifi\":\"" + (apMode ? String("AP-mode") : WiFi.SSID()) + "\"";
    j += ",\"rssi\":" + String(apMode ? 0 : WiFi.RSSI());
    j += ",\"ip\":\"" + (apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString()) + "\"";
    j += ",\"ten\":" + String(cfg.traccarEnabled ? "true" : "false");
    j += ",\"postCode\":" + String(lastPostCode);
    j += ",\"via\":\"" + lastVia + "\"";
    j += ",\"okAgo\":" + String(lastOkMs ? (int)((millis() - lastOkMs) / 1000) : -1);
    j += ",\"postAgo\":" + String(lastPostMs ? (int)((millis() - lastPostMs) / 1000) : -1);
    j += ",\"okCount\":" + String(rtcPostOk) + ",\"failCount\":" + String(rtcPostFail);
    j += ",\"fw\":\"" FW_VERSION "\",\"ota\":\"" + otaStatus + "\"";
    j += ",\"did\":\"" + cfg.deviceId + "\",\"thost\":\"" + cfg.traccarHost + "\",\"tport\":" + String(cfg.traccarPort);
    j += ",\"rsec\":" + String(cfg.reportSec) + ",\"pmin\":" + String(cfg.parkMin) + ",\"pth\":" + String(cfg.powerThreshMv);
    j += ",\"dsleep\":" + String(cfg.deepSleep ? "true" : "false");
    j += ",\"pcell\":" + String(cfg.preferCell ? "true" : "false");
    j += ",\"up\":" + String(millis() / 1000);
    j += "}";
    server.send(200, "application/json", j);
}
void handleTrack() {
    String j = "[";
    for (int i = 0; i < trackN; i++) {
        int idx = (trackHead - trackN + i + TRACK_MAX) % TRACK_MAX;
        if (i) j += ",";
        j += "[" + String(track[idx].lat, 6) + "," + String(track[idx].lon, 6) + "]";
    }
    j += "]";
    server.send(200, "application/json", j);
}
void handleOta() {
    if (apMode || WiFi.status() != WL_CONNECTED) { server.send(200, "text/plain", "OTA needs WiFi (not available in AP or 4G mode)."); return; }
    if (cfg.otaRepo.length() == 0) { server.send(200, "text/plain", "Set the GitHub repo (owner/repo) in config first."); return; }
    String url = "https://github.com/" + cfg.otaRepo + "/releases/latest/download/" + cfg.otaAsset;
    server.send(200, "text/html",
        "<meta http-equiv=refresh content='60;url=/'><body style='font:16px system-ui;background:#0e1116;color:#e6edf3;padding:30px'>"
        "Updating from<br><code>" + url + "</code><br><br>Downloading &amp; flashing… device reboots if successful (~30–60 s), then this returns to the dashboard.</body>");
    delay(200);
    otaStatus = "updating";
    WiFiClientSecure client; client.setInsecure();               // GitHub redirects to a signed host; skip cert pinning
    httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    httpUpdate.rebootOnUpdate(true);
    t_httpUpdate_return ret = httpUpdate.update(client, url);     // reboots into new image on success
    if (ret == HTTP_UPDATE_FAILED)      otaStatus = "failed: " + String(httpUpdate.getLastError()) + " " + httpUpdate.getLastErrorString();
    else if (ret == HTTP_UPDATE_NO_UPDATES) otaStatus = "no image found at URL";
    Serial.printf("OTA: %s\n", otaStatus.c_str());
}
void handleSig() {
    String j = "[";
    for (int i = 0; i < sigN; i++) { int idx = (sigHead - sigN + i + SIG_MAX) % SIG_MAX; if (i) j += ","; j += String(sigHist[idx]); }
    j += "]";
    server.send(200, "application/json", j);
}
void handleTest4g() {
    if (!haveFix()) { server.send(200, "text/plain", "No GPS fix yet - nothing to send."); return; }
    String d = reportCellular();
    server.send(200, "text/plain", "4G test -> " + d + "\n(HTTPACTION 200 = Traccar accepted)\nURL: " + buildTraccarUrl());
}
void handleConfig() {
    String p = FPSTR(PAGE_CONFIG);
    p.replace("%SSID%",  cfg.wifiSsid);   p.replace("%PASS%",  cfg.wifiPass);
    p.replace("%THOST%", cfg.traccarHost); p.replace("%TPORT%", String(cfg.traccarPort));
    p.replace("%DID%",   cfg.deviceId);
    p.replace("%RSEC%",  String(cfg.reportSec));
    p.replace("%PMIN%",  String(cfg.parkMin));
    p.replace("%PTH%",   String(cfg.powerThreshMv));
    p.replace("%TEN%",   cfg.traccarEnabled ? "checked" : "");
    p.replace("%DSLEEP%",cfg.deepSleep ? "checked" : "");
    p.replace("%APN%",   cfg.apn);   p.replace("%APNU%", cfg.apnUser); p.replace("%APNP%", cfg.apnPass);
    p.replace("%PIN%",   cfg.simPin);
    p.replace("%CELL%",  cfg.cellEnabled ? "checked" : "");
    p.replace("%PCELL%", cfg.preferCell ? "checked" : "");
    p.replace("%FWVER%", FW_VERSION);
    p.replace("%OREPO%", cfg.otaRepo); p.replace("%OASSET%", cfg.otaAsset);
    server.send(200, "text/html", p);
}
void handleSave() {
    if (server.hasArg("ssid"))  cfg.wifiSsid    = server.arg("ssid");
    if (server.hasArg("pass"))  cfg.wifiPass    = server.arg("pass");
    if (server.hasArg("thost")) cfg.traccarHost = server.arg("thost");
    if (server.hasArg("tport")) cfg.traccarPort = server.arg("tport").toInt();
    if (server.hasArg("did"))   cfg.deviceId    = server.arg("did");
    if (server.hasArg("rsec"))  cfg.reportSec   = max(5, (int)server.arg("rsec").toInt());
    if (server.hasArg("pmin"))  cfg.parkMin     = max(1, (int)server.arg("pmin").toInt());
    if (server.hasArg("pth"))   cfg.powerThreshMv = constrain((int)server.arg("pth").toInt(), 3000, 5000);
    if (server.hasArg("apn"))   cfg.apn      = server.arg("apn");
    if (server.hasArg("apnu"))  cfg.apnUser  = server.arg("apnu");
    if (server.hasArg("apnp"))  cfg.apnPass  = server.arg("apnp");
    if (server.hasArg("pin"))    cfg.simPin   = server.arg("pin");
    if (server.hasArg("orepo"))  cfg.otaRepo  = server.arg("orepo");
    if (server.hasArg("oasset")) cfg.otaAsset = server.arg("oasset");
    cfg.traccarEnabled = server.hasArg("ten");
    cfg.deepSleep      = server.hasArg("dsleep");
    cfg.cellEnabled    = server.hasArg("cell");
    cfg.preferCell     = server.hasArg("pcell");
    saveConfig();
    server.send(200, "text/html",
        "<meta http-equiv=refresh content='4;url=/'><body style='font:16px system-ui;background:#0e1116;color:#e6edf3;padding:30px'>Saved. Rebooting… <a style=color:#58a6ff href='/'>back</a></body>");
    delay(600); ESP.restart();
}

void startNetwork()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(cfg.wifiSsid.c_str(), cfg.wifiPass.c_str());
    uint32_t end = millis() + 15000;
    while (WiFi.status() != WL_CONNECTED && millis() < end) delay(300);
    if (WiFi.status() == WL_CONNECTED) {
        apMode = false;
        Serial.printf("WiFi: %s\n", WiFi.localIP().toString().c_str());
    } else {
        apMode = true; WiFi.mode(WIFI_AP); WiFi.softAP("TTGO-GPS-Setup");
        Serial.printf("WiFi: AP mode %s\n", WiFi.softAPIP().toString().c_str());
    }
    if (MDNS.begin("ttgo-gps")) Serial.println("mDNS: http://ttgo-gps.local");
    server.on("/", [](){ server.send_P(200, "text/html", PAGE_STATUS); });
    server.on("/config", handleConfig);
    server.on("/save", HTTP_POST, handleSave);
    server.on("/api/status", handleStatus);
    server.on("/api/track", handleTrack);
    server.on("/api/sig", handleSig);
    server.on("/test4g", handleTest4g);
    server.on("/ota", handleOta);
    server.begin();
}

void bootModem()
{
    pinMode(BOARD_POWERON_PIN, OUTPUT); digitalWrite(BOARD_POWERON_PIN, HIGH);
    pinMode(MODEM_RESET_PIN, OUTPUT);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL); delay(100);
    digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL);  delay(2600);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
    pinMode(MODEM_DTR_PIN, OUTPUT); digitalWrite(MODEM_DTR_PIN, LOW);
    pinMode(BOARD_PWRKEY_PIN, OUTPUT);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);  delay(100);
    digitalWrite(BOARD_PWRKEY_PIN, HIGH); delay(1000);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);
}
void startGNSS()
{
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
    delay(4000);
    for (int i = 0; i < 20; i++) if (atCmd("AT", 700).indexOf("OK") >= 0) break; else delay(500);
    atCmd("AT+CGNSSPWR=1", 3000);
    uint32_t end = millis() + 12000; String r;
    while (millis() < end) { while (SerialAT.available()) r += (char)SerialAT.read(); if (r.indexOf("READY") >= 0) break; }
    // Position is polled on demand via AT+CGNSSINFO (no NMEA streaming) so the UART
    // stays free for HTTP/status AT commands and the fix isn't torn down each report.
}

String buildTraccarUrl()
{
    String url = "http://" + cfg.traccarHost + ":" + String(cfg.traccarPort) + "/?id=" + cfg.deviceId;
    url += "&lat=" + String(fix.lat, 6) + "&lon=" + String(fix.lon, 6);
    if (fix.Y > 0)
        url += "&timestamp=" + String(toEpoch(fix.Y, fix.Mo, fix.D, fix.h, fix.m, fix.s)) + "000";
    url += "&speed=" + String(fix.speedKn, 2);
    url += "&bearing=" + String(fix.course, 1);
    url += "&altitude=" + String(fix.alt, 1);
    url += "&hdop=" + String(fix.hdop, 1);
    url += "&batt=" + String(battMv);
    return url;
}

void reportWiFi()
{
    HTTPClient http; http.begin(buildTraccarUrl());
    int c = http.POST(""); recordPost(c, "WiFi");
    Serial.printf(">> Traccar (WiFi) HTTP %d\n", c);
    http.end();
}

// POST over 4G using the A7670 HTTP AT command stack. Stops the NMEA stream for
// the duration so the HTTP URC isn't lost amid $Gx sentences. Returns a diag string.
String reportCellular()
{
    String diag;
    diag += "cgdcont[" + atCmd(("AT+CGDCONT=1,\"IP\",\"" + cfg.apn + "\"").c_str(), 1000) + "]";
    diag += " cgact[" + atCmd("AT+CGACT=1,1", 6000) + "]";
    diag += " cgatt[" + afterKey(atCmd("AT+CGATT?", 1500), "+CGATT:") + "]";
    diag += " ip[" + afterKey(atCmd("AT+CGPADDR=1", 1500), "+CGPADDR:") + "]";

    atCmd("AT+HTTPTERM", 600);                                     // clear any stale session
    diag += " init[" + atCmd("AT+HTTPINIT", 3000) + "]";
    atCmd("AT+HTTPPARA=\"CID\",1", 800);
    diag += " para[" + atCmd(("AT+HTTPPARA=\"URL\",\"" + buildTraccarUrl() + "\"").c_str(), 1500) + "]";

    String r = atCmd("AT+HTTPACTION=0", 2000);                     // 0 = GET (seed r; URC may arrive here)
    uint32_t end = millis() + 45000;
    while (millis() < end) {
        while (SerialAT.available()) r += (char)SerialAT.read();
        if (r.indexOf("+HTTPACTION:") >= 0) break;
    }
    int code = 0, i = r.indexOf("+HTTPACTION:");
    String urc = (i >= 0) ? r.substring(i, min((int)r.length(), i + 40)) : "(no URC)";
    if (i >= 0) { String seg = r.substring(i); int c1 = seg.indexOf(','), c2 = seg.indexOf(',', c1 + 1);
                  if (c1 > 0 && c2 > c1) code = seg.substring(c1 + 1, c2).toInt(); }
    atCmd("AT+HTTPTERM", 800);
    recordPost(code, "4G");
    diag += " action-urc[" + urc + "] code=" + String(code);
    diag.replace("\r", " "); diag.replace("\n", " ");
    Serial.printf(">> Traccar (4G) %s\n", diag.c_str());
    return diag;
}

void report()
{
    if (!haveFix()) return;
    bool wifiUp = !apMode && WiFi.status() == WL_CONNECTED;
    if (cfg.preferCell && cfg.cellEnabled) reportCellular();       // force 4G even on WiFi
    else if (wifiUp)                       reportWiFi();           // WiFi preferred (cheap)
    else if (cfg.cellEnabled)              reportCellular();       // fall back to 4G
}

// wait up to timeoutMs for a fresh fix, feeding the NMEA parser
bool waitForFix(uint32_t timeoutMs)
{
    uint32_t end = millis() + timeoutMs;
    while (millis() < end) {
        pollGnss();
        if (haveFix()) return true;
        server.handleClient();
        delay(700);
    }
    return haveFix();
}

void enterParkSleep()
{
    modeStr = "PARK-SLEEP";
    Serial.printf("PARK: sleeping %u min\n", cfg.parkMin);
    // power down radios to actually save energy
    atCmd("AT+CGNSSPWR=0", 1000);
    atCmd("AT+CPOF", 2000);            // modem off (PWRKEY sequence re-boots it on wake)
    WiFi.disconnect(true); WiFi.mode(WIFI_OFF);
    esp_sleep_enable_timer_wakeup((uint64_t)cfg.parkMin * 60ULL * 1000000ULL);
    delay(50);
    esp_deep_sleep_start();            // wakes into setup() again
}

void setup()
{
    Serial.begin(115200); delay(300);
    Serial.println("\n===== TTGO GPS car tracker =====");
    analogSetPinAttenuation(BOARD_BAT_ADC_PIN, ADC_11db);
    loadConfig();
    updatePower();
    Serial.printf("battery=%u mV -> power %s (threshold %u)\n",
                  battMv, powerPresent ? "PRESENT" : "ABSENT", cfg.powerThreshMv);

    bootModem();
    startGNSS();

    startNetwork();
    if (!powerPresent && cfg.deepSleep) {
        // PARK + deep-sleep: one fix, report, then sleep.
        modeStr = "PARK";
        Serial.println("PARK: acquiring fix, report, then deep-sleep...");
        if (waitForFix(120000) && cfg.traccarEnabled) { pushTrack(fix.lat, fix.lon); report(); }
        else Serial.println("PARK: no fix within window");
        enterParkSleep();               // does not return
    }
    // Otherwise stay awake (TRIP, or PARK with deep-sleep disabled) and serve the web UI.
    modeStr = powerPresent ? "TRIP" : "PARK";
    Serial.printf("%s: awake, reporting on interval (deep-sleep %s)\n", modeStr, cfg.deepSleep ? "on" : "off");
}

void loop()
{
    server.handleClient();

    static uint32_t lastGnss = 0, lastReport = 0, lastTrack = 0, lastPwr = 0, lastLog = 0, lastCell = 0;
    static uint32_t powerLostSince = 0;

    if (millis() - lastGnss > 2000) { lastGnss = millis(); pollGnss(); }
    if (millis() - lastCell > 20000) { lastCell = millis(); pollModemStatus(); }

    if (millis() - lastPwr > 5000) {          // re-check power every 5s
        lastPwr = millis();
        updatePower();
        if (powerPresent) { modeStr = "TRIP"; powerLostSince = 0; }
        else {
            if (!powerLostSince) powerLostSince = millis();
            // only deep-sleep if enabled, and after 60s without power (ignore cranking dips)
            if (cfg.deepSleep && millis() - powerLostSince > 60000) {
                Serial.println("Power lost >60s -> PARK deep-sleep");
                if (haveFix() && cfg.traccarEnabled) report();
                enterParkSleep();          // does not return
            }
            modeStr = "PARK";              // awake PARK when deep-sleep is off
        }
    }

    if (haveFix() && millis() - lastTrack > 5000) { lastTrack = millis(); pushTrack(fix.lat, fix.lon); }
    // TRIP reports every reportSec; PARK (awake) every parkMin
    uint32_t interval = powerPresent ? (uint32_t)cfg.reportSec * 1000UL : (uint32_t)cfg.parkMin * 60000UL;
    if (haveFix() && cfg.traccarEnabled && millis() - lastReport > interval) { lastReport = millis(); report(); }
    if (millis() - lastLog > 5000) {
        lastLog = millis();
        Serial.printf("[%s] batt:%umV pwr:%d wifi:%s sats:%d %s\n",
                      modeStr, battMv, powerPresent, apMode ? "AP" : "STA",
                      fix.sats, haveFix() ? "FIX" : "NO-FIX");
    }
}
