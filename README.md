# lilygo-gps

GPS car tracker firmware for the **LILYGO T-A7670G/E/SA R2** (ESP32-WROVER-E + A7670E-FASE 4G modem with built-in GNSS).

## Features
- GPS fix from the modem's built-in GNSS (`AT+CGNSS…`).
- Uploads position to a **Traccar** server (OsmAnd protocol) — **WiFi preferred, 4G fallback**.
- **Onboard web UI** (`http://ttgo-gps.local`): live map + trail, status, and a config page (WiFi, Traccar host/port/device-id, cellular APN/PIN, reporting intervals, power threshold, OTA).
- **AP fallback** hotspot `TTGO-GPS-Setup` when no known WiFi.
- **TRIP / PARK power logic**: reports frequently on external power, deep-sleeps between reports on battery.
- **Upload health** (last success, OK/fail counts) + **4G signal trend** on the dashboard.
- **OTA updates** from this repo's GitHub Releases (`/config` → Update firmware).

## Build & flash
```bash
pio run -e ta7670g -t upload --upload-port /dev/cu.usbserial-XXXX
```

## OTA release flow
Bump `FW_VERSION` in `src/main.cpp`, then:
```bash
git tag v1.0.1 && git push --tags
```
The GitHub Action builds `firmware.bin` and attaches it to a release; the device pulls the latest release from `/config` → **Update firmware now**.

## Hardware notes
- Modem is **A7670E-FASE** (has built-in GNSS) despite the "A7670G" label — no external L76K.
- Board pins: POWERON 12, RESET 5, PWRKEY 4 (~1 s pulse), modem UART TX 26 / RX 27; battery ADC 35.
- GPS antenna → GPS socket; LTE antenna → LTE/MAIN socket (u.FL / MHF1).
