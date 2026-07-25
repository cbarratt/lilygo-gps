#!/usr/bin/env python3
"""
HTTP -> MQTT bridge for the LilyGO GPS tracker + Home Assistant.

The device POSTs a small JSON status here every report tick (battery-friendly,
no persistent connection). This bridge:
  - republishes to MQTT with Home Assistant DISCOVERY, so the tracker shows up
    as one device with a map marker + sensors + online/offline, no YAML.
  - exposes HA buttons (reboot / report / test4g) whose presses are QUEUED and
    returned to the device in the HTTP response (poll-based remote control).

Run on your always-on home box (same one as HA/Mosquitto).
Deps:  pip install paho-mqtt
Config: env vars (see below) or edit the defaults.

  MQTT_HOST=127.0.0.1  MQTT_PORT=1883  MQTT_USER=...  MQTT_PASS=...  HTTP_PORT=5057

Point the device's /config Heartbeat URL at:  http://<this-host>:5057/hb
(from 4G it must be internet-reachable, e.g. http://home.barratt.me:5057/hb)
"""
import os, json, time, threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import paho.mqtt.client as mqtt

MQTT_HOST = os.environ.get("MQTT_HOST", "127.0.0.1")
MQTT_PORT = int(os.environ.get("MQTT_PORT", "1883"))
MQTT_USER = os.environ.get("MQTT_USER", "")
MQTT_PASS = os.environ.get("MQTT_PASS", "")
HTTP_PORT = int(os.environ.get("HTTP_PORT", "5057"))
OFFLINE_AFTER = int(os.environ.get("OFFLINE_AFTER", "300"))   # secs w/o heartbeat -> offline

cmd_queue = {}        # device_id -> [commands]
last_seen = {}        # device_id -> epoch
discovered = set()
lock = threading.Lock()

mc = mqtt.Client()
if MQTT_USER:
    mc.username_pw_set(MQTT_USER, MQTT_PASS)

def base(did):
    return "".join(c if c.isalnum() else "_" for c in did)

def publish_discovery(did):
    nid = base(did)
    dev = {"identifiers": [nid], "name": did, "manufacturer": "LilyGO", "model": "T-A7670G tracker"}
    avail = f"tracker/{nid}/availability"

    def cfg(comp, key, extra):
        payload = {"unique_id": f"{nid}_{key}", "device": dev,
                   "availability_topic": avail, "state_topic": f"tracker/{nid}/state",
                   "value_template": "{{ value_json.%s }}" % key}
        payload.update(extra)
        mc.publish(f"homeassistant/{comp}/{nid}/{key}/config", json.dumps(payload), retain=True)

    # location on the HA map
    mc.publish(f"homeassistant/device_tracker/{nid}/loc/config", json.dumps({
        "unique_id": f"{nid}_loc", "device": dev, "availability_topic": avail,
        "json_attributes_topic": f"tracker/{nid}/attrs", "source_type": "gps"}), retain=True)
    # sensors
    cfg("sensor", "batt",   {"name": "Battery", "unit_of_measurement": "mV", "icon": "mdi:battery"})
    cfg("sensor", "sats",   {"name": "Satellites", "icon": "mdi:satellite-variant"})
    cfg("sensor", "hdop",   {"name": "HDOP", "icon": "mdi:crosshairs-gps"})
    cfg("sensor", "signal", {"name": "4G signal", "unit_of_measurement": "dBm", "icon": "mdi:signal"})
    cfg("sensor", "mode",   {"name": "Mode", "icon": "mdi:car"})
    cfg("sensor", "up",     {"name": "Uptime", "unit_of_measurement": "s", "icon": "mdi:timer"})
    cfg("binary_sensor", "fix", {"name": "GPS fix", "payload_on": "True", "payload_off": "False",
                                 "device_class": "connectivity"})
    # command buttons -> publish to cmd topic
    for c in ("reboot", "report", "test4g"):
        mc.publish(f"homeassistant/button/{nid}/{c}/config", json.dumps({
            "unique_id": f"{nid}_{c}", "device": dev, "name": c.capitalize(),
            "command_topic": f"tracker/{nid}/cmd", "payload_press": c}), retain=True)

def on_connect(cl, ud, flags, rc):              # (re)subscribe to every device's cmd topic
    print("MQTT connected rc=", rc)
    cl.subscribe("tracker/+/cmd")

def on_message(cl, ud, msg):                    # HA button press -> queue a command
    did = msg.topic.split("/")[1]
    with lock:
        cmd_queue.setdefault(did, []).append(msg.payload.decode())
    print("queued cmd", did, msg.payload.decode())

def offline_watch():
    while True:
        now = time.time()
        for did, t in list(last_seen.items()):
            if now - t > OFFLINE_AFTER:
                mc.publish(f"tracker/{base(did)}/availability", "offline", retain=True)
        time.sleep(30)

class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        try:
            d = json.loads(self.rfile.read(n) or b"{}")
        except Exception:
            self.send_response(400); self.end_headers(); return
        did = str(d.get("id", "tracker")); nid = base(did)
        with lock:
            if did not in discovered:
                publish_discovery(did); discovered.add(did)
            last_seen[did] = time.time()
        mc.publish(f"tracker/{nid}/availability", "online", retain=True)
        mc.publish(f"tracker/{nid}/state", json.dumps(d), retain=True)
        if d.get("fix"):
            mc.publish(f"tracker/{nid}/attrs", json.dumps(
                {"latitude": d.get("lat"), "longitude": d.get("lon"), "gps_accuracy": d.get("hdop", 0)}), retain=True)
        # return a queued command, if any
        with lock:
            q = cmd_queue.get(did) or []
            reply = {"cmd": q.pop(0)} if q else {"ok": 1}
        body = json.dumps(reply).encode()
        self.send_response(200); self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)

if __name__ == "__main__":
    mc.on_connect = on_connect
    mc.on_message = on_message
    mc.reconnect_delay_set(min_delay=1, max_delay=30)
    mc.connect_async(MQTT_HOST, MQTT_PORT, 60)     # non-blocking; auto-reconnects if broker is down
    mc.loop_start()
    threading.Thread(target=offline_watch, daemon=True).start()
    print(f"HA bridge on :{HTTP_PORT}  ->  MQTT {MQTT_HOST}:{MQTT_PORT}")
    ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), H).serve_forever()
