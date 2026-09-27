# lilygo-gps

GPS car tracker firmware for the **LILYGO T-A7670G/E/SA R2** (ESP32-WROVER-E + A7670E-FASE 4G modem with built-in GNSS), plus a small **Home Assistant bridge**.

- **Firmware** (`src/main.cpp`) — GNSS + WiFi/4G uplink to Traccar, onboard web UI, TRIP/PARK driven by an LIS3DH accelerometer, deep sleep, OTA.
- **HA bridge** (`ha_bridge.py`) — an HTTP→MQTT bridge that turns the device's status heartbeats into a Home Assistant device (map + sensors + buttons) and relays remote commands back.

---

## Hardware

- Modem is **A7670E-FASE** (has built-in GNSS) despite the "A7670G" label — there is **no** external L76K. GPS comes from the modem via `AT+CGNSSINFO`.
- Pins: POWERON 12, RESET 5, PWRKEY 4 (needs a ~1 s pulse to boot), modem UART TX 26 / RX 27 @115200, battery ADC on GPIO35, BOOT button GPIO0.
- Antennas: **active GPS** antenna → GPS socket (board supplies bias; hugely better than the ceramic patch), **LTE** antenna → LTE/MAIN u.FL socket.
- Power in: **USB-C 5 V** (charges the 18650) or the **18650** itself. **Not 12 V** — for a car feed, drop 12 V→5 V with a fused automotive buck into USB-C, ideally off a **switched/ignition** line so it charges while driving without draining the car battery when parked. (Charging no longer affects the mode — movement does.)
- **Optional LIS3DH accelerometer** (Adafruit breakout) for motion wake — see [Motion wake](#motion-wake-lis3dh). Firmware auto-detects it; without it everything behaves as before.

  | LIS3DH | → LILYGO | Why |
  |--------|----------|-----|
  | VIN | 3V3 | 3.3 V so INT is a 3.3 V logic level |
  | GND | GND | |
  | SDA | GPIO21 | free I²C pair |
  | SCL | GPIO22 | |
  | INT1 | **GPIO32** | must be an RTC GPIO to wake from deep sleep |

  Leave SDO/CS/A1–A3 unconnected (I²C address 0x18). The breakout's STEMMA QT socket doesn't carry INT1, so it has to be wired from the header pads.

---

## Modes

The device is always in one of two operating modes:

| Mode | When (Auto) | Behaviour |
|------|------|-----------|
| **TRIP** | sustained movement (LIS3DH) | stays awake, reports every `Trip interval` seconds (default 10 s) |
| **PARK** | no movement for ~4 min | reports its position **once**, then deep-sleeps; wakes for check-ins every `Park check-in interval`, or on movement |

**Movement is the only automatic trigger.** Battery voltage is reported as telemetry but does **not** decide the mode. Earlier firmware tried to detect the ignition from charging voltage, but a charging cell's voltage overlaps its normal discharge range (and this board has no software-readable VBUS), so it was never reliable and was removed in v1.13.0. Without an LIS3DH fitted, Auto never enters TRIP by itself — use Force TRIP.

### Motion wake (LIS3DH)

With the accelerometer fitted, the LIS3DH's own interrupt engine watches for movement (high-pass filtered, so gravity and tilt don't count) and wakes the ESP32 from deep sleep on GPIO32. The sensor draws ~3 µA and the ESP32 does no polling while asleep.

- **Confirm before TRIP.** A motion wake keeps the radios **off** and watches for up to 15 s; it only switches to TRIP if it sees movement in **3 separate seconds**. Driving passes in ~2 s. A door slam or someone leaning on the car doesn't pass: that's a **nudge**, and it goes straight back to sleep without booting the modem. Modem power-up is what browns the board out on battery, so this matters.
- **Hold.** Once in TRIP, every movement event refreshes a **3-minute hold**. After the car stops moving it waits out the hold plus a 60 s debounce (~4 min total), sends one final position report, then deep-sleeps.
- **Reset safety net.** Powering the modem up on battery can brown the board out and reset it, which used to wipe the motion hold, so it parked mid-drive. A motion TRIP is now flagged in flash before the modem starts. If the board resets during TRIP, it comes back in TRIP (`wake: resume`) instead of parking. It gives up after 3 back-to-back resumes, so a reset loop can't drain the battery. The count clears after 2 stable minutes in TRIP.
- **Auto only.** Force PARK doesn't arm motion wake; Force TRIP ignores it.
- **Tuning.** `Motion wake threshold` (default 80 mg). Watch `nudges` in the heartbeat: if it climbs while the car sits parked, raise the threshold (each nudge costs a ~15 s radio-off wake). If pulling away doesn't flip it to TRIP, lower it.
- **Bench check.** `/api/status` shows `imu:true` and `acc:[x,y,z]` in mg. Lying flat, that should read roughly `[0,0,1000]`.

### Manual mode override

`Mode override` on `/config` (and HA buttons) forces the mode regardless of movement:

- **Auto** — motion decides (default).
- **Force TRIP** — always awake + frequent reports. Uses battery (~85 mA / ~1.5 days). Good for live tracking on demand, or if no LIS3DH is fitted.
- **Force PARK** — deep-sleep park even while moving; motion wake is not armed. Parks promptly (skips the 60 s debounce).

The override **persists** (NVS) until changed. The status page and the HA `Mode override` sensor show the current value.

### Deep sleep (PARK)

`Deep-sleep when on battery` (default off). When **on** and parked, the device powers the modem/radios down and sleeps, giving **weeks** of battery (measured parked draw ≈ 1–2 mA, ~0.5 mV/h) vs ~1.5 days awake. When **off**, it stays awake in PARK (reachable, but ~85 mA).

While parked in deep sleep it only wakes for three reasons:

1. **Park entry (once)** — on parking: GPS fix + position report to Traccar and HA. It tries for a fresh fix for up to `Park fix window` s (default 300); if it can't, it reports the **cached last-known position**, tagged `cached` in HA.
2. **Check-in** — every `Park check-in interval` s (`cmd`): a **status-only** heartbeat (battery, no position) that also collects remote commands. Over home WiFi the modem stays off; away from WiFi it has to wake the modem to use 4G. `0` = no timed check-ins at all — it sleeps until moved or BOOT is pressed (if no LIS3DH is fitted it falls back to hourly, so it can never become unreachable).
3. **Movement** — see [Motion wake](#motion-wake-lis3dh). An unconfirmed nudge goes straight back to sleep and keeps the same check-in schedule (it doesn't restart the countdown).

There's no longer a 60 s "ignition check" tick — that existed only to read charging voltage — so between check-ins the ESP32 sleeps continuously.

### Waking a sleeping device

You **cannot push** to a deep-sleeping device — it only wakes on its check-in timer, movement, or the BOOT button; a queued command is collected at the next wake that checks in.

- **BOOT button** — wakes it and keeps it awake 5 min (web UI reachable). Also the reliable way to get it OTA-able.
- **Movement** (LIS3DH fitted, Auto mode) — instant wake; TRIP after ~2 s of sustained motion.
- **HA `Wake to TRIP` / `Force TRIP`** — queued and collected on the next heartbeat: within seconds in TRIP, within `Park check-in interval` in PARK (or when next moved, if it's 0).

---

## `/config` reference

| Field | Key | Default | Meaning |
|-------|-----|---------|---------|
| WiFi SSID / pass | `ssid` `pass` | from `.env` | home WiFi (STA) |
| Traccar host / port / device id | `thost` `tport` `did` | — | OsmAnd endpoint |
| Mode override | `mode` | Auto | Auto / Force TRIP / Force PARK |
| Trip interval (s) | `rsec` | 10 | report cadence while moving |
| Park fix window (s) | `pfix` | 300 | max wait for a fresh fix before using cached |
| Park check-in interval (s) | `cmd` | 0 | status + remote-command check-in while parked (0 = motion/BOOT only) |
| Motion wake threshold (mg) | `mth` | 80 | LIS3DH movement threshold (16 mg steps) |
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
