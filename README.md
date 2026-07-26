# lilygo-gps

GPS car tracker firmware for the **LILYGO T-A7670G/E/SA R2** (ESP32-WROVER-E + A7670E-FASE 4G modem with built-in GNSS), plus a small **Home Assistant bridge**.

- **Firmware** (`src/main.cpp`) — GNSS + WiFi/4G uplink to Traccar, onboard web UI, TRIP/PARK power management with deep sleep, OTA.
- **HA bridge** (`ha_bridge.py`) — an HTTP→MQTT bridge that turns the device's status heartbeats into a Home Assistant device (map + sensors + buttons) and relays remote commands back.

---

## Hardware

- Modem is **A7670E-FASE** (has built-in GNSS) despite the "A7670G" label — there is **no** external L76K. GPS comes from the modem via `AT+CGNSSINFO`.
- Pins: POWERON 12, RESET 5, PWRKEY 4 (needs a ~1 s pulse to boot), modem UART TX 26 / RX 27 @115200, battery ADC on GPIO35, BOOT button GPIO0.
- Antennas: **active GPS** antenna → GPS socket (board supplies bias; hugely better than the ceramic patch), **LTE** antenna → LTE/MAIN u.FL socket.
- Power in: **USB-C 5 V** (charges the 18650) or the **18650** itself. **Not 12 V** — for a car feed, drop 12 V→5 V with a fused automotive buck into USB-C, ideally off a **switched/ignition** line so "charging" == "ignition on".

---

## Modes & power logic

The device is always in one of two operating modes:

| Mode | When | Behaviour |
|------|------|-----------|
| **TRIP** | external power present (charging) | stays awake, reports every `Trip interval` seconds (default 10 s) |
| **PARK** | on battery | reports every `Park interval` minutes (default 45); deep-sleeps between reports if enabled |

**Power detection** is a heuristic on the battery ADC: `power present` when `battMv >= Power-detect threshold` (default 4200 mV) — i.e. the charger is holding the rail up. It is inherently fuzzy (charging ~4.2 V overlaps a full battery ~4.16 V); the manual override and (future) IMU exist to sidestep it.

### Manual mode override

`Mode override` on `/config` (and HA buttons) forces the mode regardless of power:

- **Auto** — power-detect decides (default).
- **Force TRIP** — always awake + frequent reports, ignore power. Uses battery (~85 mA / ~1.5 days). Good for "I'm driving but it read PARK".
- **Force PARK** — deep-sleep park even while charging. Parks promptly (skips the crank-dip debounce).

The override **persists** (NVS) until changed. The status page and the HA `Mode override` sensor show the current value.

### Deep sleep (PARK)

`Deep-sleep when on battery` (default off). When **on** and parked, the device powers the modem/radios down and sleeps, giving **weeks** of battery (measured parked draw ≈ 1–2 mA, ~0.5 mV/h) vs ~1.5 days awake. When **off**, it stays awake in PARK (reachable, but ~85 mA).

Deep-sleep PARK uses three cadences so it can be responsive **and** frugal:

1. **Ignition-check** — every `Ignition-check interval` s (default 60), it wakes *cheaply* (just reads the battery ADC, **no modem**) to see if the charger came on. If so → TRIP within ~60 s. Costs ~0.4 mA.
2. **Command-check** — every `Command-check interval in PARK` s (`cmdSec`, default 0 = off), it wakes and does a **modem + heartbeat only, no GPS fix** to collect remote commands (Force TRIP / Wake) faster than the full report. **Each check wakes the modem (~1–2 mAh)** — keep it ≥600 or 0 when running deep sleep for weeks; in awake-PARK it's ~free.
3. **Full report** — every `Park interval` minutes: modem + A-GPS + GPS fix + report to Traccar + HA heartbeat, then back to sleep.

On each full report it tries for a fresh fix for up to `Park fix window` s (default 300); if it can't, it reports the **cached last-known parked position** (a parked car hasn't moved), tagged `cached` in HA so you can tell.

### Waking a sleeping device

You **cannot push** to a deep-sleeping device — it only wakes on its timer, the BOOT button, or (once awake) a queued command.

- **BOOT button** — wakes it and keeps it awake 5 min (web UI reachable). Also the reliable way to get it OTA-able.
- **Ignition (charging)** — caught within `Ignition-check interval` (~60 s) → TRIP.
- **HA `Wake to TRIP` / `Force TRIP`** — queued and collected on the next heartbeat: seconds in TRIP, `cmdSec` in PARK (or the full park interval if `cmdSec` = 0).

---

## `/config` reference

| Field | Key | Default | Meaning |
|-------|-----|---------|---------|
| WiFi SSID / pass | `ssid` `pass` | from `.env` | home WiFi (STA) |
| Traccar host / port / device id | `thost` `tport` `did` | — | OsmAnd endpoint |
| Mode override | `mode` | Auto | Auto / Force TRIP / Force PARK |
| Trip interval (s) | `rsec` | 10 | report cadence on power |
| Park interval (min) | `pmin` | 45 | report/sleep cadence on battery |
| Park fix window (s) | `pfix` | 300 | max wait for a fresh fix before using cached |
| Ignition-check interval (s) | `chk` | 60 | cheap ADC wake cadence to catch the charger |
| Command-check interval in PARK (s) | `cmd` | 0 | remote-command poll while parked (0 = off) |
| Power-detect threshold (mV) | `pth` | 4200 | battMv at/above = "external power" |
| Deep-sleep when on battery | `dsleep` | off | deep-sleep in PARK for weeks battery |
| Cellular: APN / user / pass / PIN | `apn` … | mobile.sky | 4G |
| Use 4G / prefer 4G | `cell` `pcell` | on / off | uplink selection |
| A-GPS | `agps` | on | download assist data for fast fixes |
| Heartbeat enabled / URL | `hben` `hburl` | off / — | HA bridge endpoint |
| OTA repo / asset | `orepo` `oasset` | — / firmware.bin | GitHub release source |

WiFi: the config hotspot `TTGO-GPS-Setup` is raised **only while STA is disconnected** (15 s debounce) and dropped when home WiFi returns — so there's no open AP at home, but it's reachable when away.

---

## Home Assistant

Run `ha_bridge.py` (Docker, `docker-compose.yml`) on the box that runs HA + Mosquitto. Point the device's `Heartbeat URL` at it (`http://<host>:5057/hb`). It publishes MQTT discovery so the device shows up with:

- **device_tracker** (map), **sensors**: battery, satellites, HDOP, 4G signal, mode, uptime, **fix source** (fresh/cached), **fix age**, **last reported**, **mode override**; **binary_sensor**: GPS fix; **availability** (online/offline).
- **Buttons** (relayed as commands on the next heartbeat): Reboot, Report, Test 4G, **Wake to TRIP**, **Mode Auto**, **Force TRIP**, **Force PARK**.

Remote commands: `wake` (10-min temporary wake), `trip`/`park`/`auto` (persistent override), `reboot`, `report`, `test4g`, `agps`.

> **HA entity_id gotcha:** HA derives the entity_id from the entity **name**, not the unique_id. "Wake to TRIP" → `button.callums_car_wake_to_trip`, "Force TRIP" → `button.callums_car_force_trip`, etc. Match dashboard cards to the real IDs.

---

## Build, flash & OTA

**USB flash** (most reliable, required when OTA is awkward):
```bash
pio run -e ta7670g -t upload --upload-port /dev/cu.usbserial-XXXX
```

**OTA release flow** — bump `FW_VERSION` in `src/main.cpp`, then:
```bash
git tag vX.Y.Z && git push origin vX.Y.Z
```
The GitHub Action builds `firmware.bin` and attaches it to a Release. The device pulls `…/releases/latest/download/firmware.bin` from `/config` → **Update firmware now** (or `GET /ota`).

> **OTA + deep sleep gotcha:** OTA runs in the main loop, so it **only completes when the device is in the awake path** — a **BOOT-wake** or **TRIP**. During a 45-min park wake the `/ota` page responds but the flash is silently dropped (setup() sleeps before loop() runs). Check `awakeLeft > 0` in `/api/status` to know it's OTA-able, or USB-flash. OTA is WiFi-only (the ESP32 IP stack; 4G goes through the modem's separate AT stack).

WiFi creds are **not** in source — blank defaults, injected at build from a gitignored `.env` via `load_env.py`.

---

## Checking it's alive

- **Web UI / status**: `curl http://<device-ip>/api/status` → JSON (`fw`, `ovr`, `power`, `battmv`, `fix`, `sats`, `via`, `hbStatus`, `awakeLeft`, …). Or open `http://ttgo-gps.local`.
- **Reachable?** No response usually = deep-sleeping (only awake ~every park interval or on BOOT).
- **HA heartbeat**: the bridge's `Last reported` sensor tracks last-received time; `Fix source` shows fresh vs cached.
