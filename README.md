# SalahSense

**A privacy-preserving, sunrise-aware, pillow-vibration alarm with on-device sensor fusion and a web console.**

---

## Why?

Phone alarms are loud, easy to snooze, and not sunrise-aware—bad news when you’re traveling or sharing a room. **SalahSense** is a tiny ESP32 device that quietly vibrates your pillow at *sunrise − offset* and stops the moment you either tap the pillow (piezo) or leave the bed (FSR). Everything runs locally: sensing, decision-making, and actuation—no cloud required.

---

## Highlights

* **Sunrise-relative alarm**: computes `alarm = sunrise − offset` on-device (with optional network fallback).
* **Multimodal sensing**: FSR (bed occupancy), piezo (tap-to-stop), BH1750 (ambient light), QMC5883L (heading/Qibla).
* **Silent & targeted**: PWM-driven vibration motor (ramp → pulse pattern) under your pillow.
* **Local web console**: calibration, status, and test tools at `http://salahsense.local` (AP or STA mode).
* **Edge-only privacy**: no long-term logs; calibration/config lives in NVS on the device.
* **Portable**: works fully offline after initial setup.

> Scope: this is a working prototype with on-device control. Results include measured timings (e.g., tap-to-stop latency) and **estimated** power/runtime.

---

## Hardware

| Part / Module                            | Qty | Notes                                                       |
| ---------------------------------------- | --: | ----------------------------------------------------------- |
| ESP32-DevKit (ESP32-WROOM)               |   1 | Edge compute + Wi-Fi AP/STA + web server                    |
| DS3231 RTC                               |   1 | Stable clock (kept in **local time** via stored UTC offset) |
| BH1750 light sensor                      |   1 | I²C ambient light                                           |
| QMC5883L magnetometer                    |   1 | I²C heading + Qibla                                         |
| FSR-402 (or similar)                     |   1 | Bed occupancy (analog)                                      |
| Piezo disc (27–35 mm)                    |   1 | Tap-to-stop (analog)                                        |
| Vibration motor (5 V)                    |   1 | Haptic actuator under pillow                                |
| 2N2222 **or** logic-level MOSFET + diode |   1 | Motor driver + flyback protection                           |
| Resistors/caps, wiring                   |   — | ~1 kΩ base/gate drive, decoupling, etc.                     |
| 5 V 2 A supply                           |   1 | Headroom for motor inrush                                   |

### Pin map (default)

* **I²C**: SDA **27**, SCL **14** → BH1750, DS3231, QMC5883L
* **FSR**: GPIO **32** (ADC1_CH4)
* **Piezo**: GPIO **33** (ADC1_CH5)
* **Motor PWM**: GPIO **26** (LEDC ch0)

**Motor wiring (BJT example):** GPIO26 → 1 kΩ → 2N2222 base; emitter→GND; collector→motor−; motor+→+5 V; **flyback diode** across motor.
(Use a logic-level MOSFET in production for lower loss.)

---

## Build & Flash

### Option A — PlatformIO (recommended)

1. Install [PlatformIO](https://platformio.org/) (VS Code extension).
2. Use the included `platformio.ini` (env `esp32dev`).
3. Plug in your ESP32, then:

   ```bash
   pio run
   pio run --target upload
   pio device monitor -b 115200
   ```

### Option B — Arduino IDE

1. Install ESP32 board support and the following libraries: `RTClib`, `BH1750`, `Preferences`, `ESPmDNS`.
2. Open the `.ino` or main `.cpp`, select **ESP32 Dev Module**, set serial to **115200**, and upload.

> **Don’t hard-code Wi-Fi secrets** in source. Use AP provisioning or NVS-backed forms in the web UI.

---

## First-Run & Calibration

1. **Power** the device; it starts in AP or STA:

   * AP SSID: `SalahSense` (default pass: `12345678`)
   * Or it will join your configured Wi-Fi (STA).
2. Visit `http://salahsense.local` (or AP IP).
3. In the web console:

   * **Clock**: set from browser or NTP; set **UTC offset** (e.g., EDT −240).
   * **Location**: enter your lat/lon (for Qibla and offline sunrise).
   * **Sunrise**: set minutes before sunrise (e.g., **20**); press **Fetch Sunrise** (optional) or rely on **offline sunrise**.
   * **FSR**: press **Calibrate: EMPTY**, then lie down and **Calibrate: OCCUPIED**; keep hysteresis near **40**.
   * **Piezo**: set threshold (**~35**) so a **single firm tap** stops the alarm; avoid false triggers.
   * **Compass**: run **Start Cal**, rotate slowly on all axes, **Stop & Save**; set magnetic declination for your locale.
   * **Test Alarm**: fire a one-shot test to confirm ramp→pulses, tap-to-stop, and bed-empty auto-stop.

---

## How It Works (short)

### Orchestration rules

* **Alarm start**: `time == (sunrise − offset)` **AND** not “I’m up” → motor **ON** (ramp → pulses)
* **Intent stop**: piezo **single tap** → motor **OFF**; set “I’m up” for 5 min
* **Auto stop**: **FSR empty** continuously ≥ 10 s during alarm → motor **OFF**
* **Test**: UI “Start” → motor **ON** (test window)
* **Safety** *(optional)*: cap max alarm duration (e.g., 3 min) → motor OFF

### Architecture (edge-only trust)

```
[Sensors]  FSR(GPIO32)  Piezo(GPIO33)  BH1750(I²C)  QMC5883L(I²C)
                    \       |            |            |
                     \      |            |            |       PWM
                      \     |            |            |    ┌─────────┐
                       └────┴────────────┴────────────┴──▶ │ Motor   │
                                    ┌──────────────────────┴─────────┐
                                    │           ESP32 (edge + GW)    │
                                    │  Web UI + REST | NVS | RTC     │
                                    │  AP/STA + mDNS | NTP | Sunrise │
                                    │  State Machine | EMA | Qibla   │
                                    └─────────────────────────────────┘
```

---

## Sunrise & Qibla

* **Sunrise**: by default, computed **on-device** (civil sunrise at −0.833°). Optionally fetch via a JSON source when online; values are cached. Alarm = **Sunrise − Offset**.
* **Qibla**: great-circle bearing from your lat/lon to the Kaaba; declination-corrected heading from the magnetometer; manual **override** can be saved to NVS when magnetic noise is high.

---

## Web API (selected)

Read-only status:

```bash
curl http://salahsense.local/api/status
```

Motor & modes:

```bash
curl http://salahsense.local/api/motor/mode/auto
curl "http://salahsense.local/api/motor/pwm?d=180"
```

FSR & piezo:

```bash
curl http://salahsense.local/api/fsr/cal/empty
curl http://salahsense.local/api/fsr/cal/occupied
curl "http://salahsense.local/api/fsr/hys?x=40"
curl "http://salahsense.local/api/pz/set_thr?thr=35"
```

Clock, sunrise:

```bash
curl "http://salahsense.local/api/rtc/ntp"
curl "http://salahsense.local/api/rtc/set_tz?min=-240"
curl "http://salahsense.local/api/sun/toggle"
curl "http://salahsense.local/api/sun/set_off?m=20"
curl "http://salahsense.local/api/sun/fetch"
```

Test & control:

```bash
curl "http://salahsense.local/api/test/in?sec=10"
curl http://salahsense.local/api/alarm/stop
curl http://salahsense.local/api/alarm/snooze
```

Save / reset:

```bash
curl http://salahsense.local/api/save
curl http://salahsense.local/api/reset
```

> **Security note:** mutations are open by default on LAN/AP. See **Security** to lock them down.

---

## Security (pragmatic)

* **Token-gate writes**: front the device with a local reverse proxy (e.g., Traefik/Caddy) and require a bearer token for `/api/*` mutations.
* **AP hygiene**: change the AP password on first boot; disable AP after STA provisioning.
* **LAN isolation**: put the device on an **IoT VLAN/SSID** and restrict inbound access.
* **Minimize data**: the status endpoint can omit sensitive fields in AP mode; the device keeps **no long-term presence logs**.

---

## Power (estimate)

* Idle (optimized duty-cycle + modem sleep): **~40–55 mA**
* Alarm motor pulses: **120–250 mA** (≈60 s typical)

**Battery runtime (estimate):** with a 2000 mAh pack (80% usable ≈ 1600 mAh) and ~9 mA average over a day,
`1600 mAh / 9 mA ≈ 177 h ≈ 7.4 days`.
One 60-s alarm/day at 220 mA adds ≈**3.7 mAh/day** (negligible vs. idle).

---

## Troubleshooting

* **Can’t reach `salahsense.local`?** Try the IP shown in the serial log or your router’s DHCP leases. Verify mDNS on your OS.
* **Alarm didn’t fire?** Check **RTC (local time)** and **UTC offset**, confirm **Sunrise** values and **Offset**, and ensure **sunrise mode** is enabled.
* **Tap doesn’t stop?** Lower or raise `PZ_THR` until a single intentional tap reliably stops the motor.
* **Bed not detected?** Re-run **FSR EMPTY/Occupied** calibration and tune hysteresis.
* **Heading off?** Re-calibrate the magnetometer away from ferrous objects; set declination; use Qibla override if needed.

---

## Safety

* Use a **flyback diode** across the motor.
* Ensure the motor and wiring are strain-relieved and thermally safe under bedding.
* Never run from an undersized USB supply.

