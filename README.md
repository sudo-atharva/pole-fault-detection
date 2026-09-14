# Pole Fault Detection

ESP32 monitors an AC line for short circuit, overvoltage, and physical
pole-down (tilt), cuts a relay on fault, and SMS-alerts a phone number with
GPS coordinates. Live readings are also served over a local WiFi AP.

## Hardware / pinout

| Signal              | ESP32 pin | Notes |
|---------------------|-----------|-------|
| Relay IN            | GPIO 25   | digital out |
| ACS712 OUT          | GPIO 34   | analog in (ADC1_CH6, input-only) |
| ZMPT101B OUT        | GPIO 35   | analog in (ADC1_CH7, input-only) — **pin assumed, not given in the original spec; move it if it's already used** |
| MPU6050 SDA         | GPIO 21   | I2C |
| MPU6050 SCL         | GPIO 22   | I2C |
| SIM800L TX → ESP32 RX1 | GPIO 27 | UART1 |
| SIM800L RX ← ESP32 TX1 | GPIO 26 | UART1, through a voltage divider (SIM800L is 3.3V logic tolerant but many boards run TX at 5V) |
| NEO-6M TX → ESP32 RX2  | GPIO 16 | UART2 |
| NEO-6M RX ← ESP32 TX2  | GPIO 17 | UART2 |

## Fault logic

Sampled every 500ms, each condition needs 3 consecutive bad samples
(debounce) before it fires:

- **Short circuit** — current RMS above `CURRENT_TRIP_A`
- **Pole down** — MPU6050 tilt away from the install-time baseline above `TILT_TRIP_DEG`
- **Overvoltage** — voltage RMS above `VOLTAGE_HIGH_V`

Any of these: relay opens (line isolated) and latches open until reboot,
an SMS goes out with the reason and a Google Maps link from the GPS fix.

Undervoltage is logged to Serial only, no trip/SMS (wasn't asked for).

## Calibration (do this before trusting it on real mains)

All in `src/main.cpp`, top of file:

- `ACS712_OFFSET_V`, `ACS712_SENS_V_PER_A` — measure your module's 0A bias and its mV/A rating (66/100/185mV/A depending on model)
- `ZMPT_OFFSET_V`, `ZMPT_SENS_V_PER_V` — measure your module's 0V bias, calibrate the sensitivity against a known mains voltage
- `CURRENT_TRIP_A`, `VOLTAGE_HIGH_V`, `TILT_TRIP_DEG` — trip thresholds for your load/site
- `RELAY_ACTIVE_LOW` — flip if the relay module energizes backwards
- `ALERT_NUMBER` — SMS recipient, E.164 format
- MPU6050 mount orientation is assumed to put its Z axis along the pole's length; if it's mounted differently the tilt math needs the axis swapped

## WiFi dashboard

Boots an AP: SSID `PoleFaultAP`, password `12345678` (edit `AP_SSID`/`AP_PASS`
in `main.cpp`). Connect and open `http://192.168.4.1/` for live voltage,
current, tilt, relay state, fault reason, and GPS.

The page (`data/index.html`) lives on LittleFS, separate from firmware.
Flash it once (and again whenever you edit it):

```
pio run -t uploadfs
```

## Debug passthrough

Anything typed into the Serial Monitor (115200 baud) is forwarded straight
to the SIM800L, so you can hand-test it with raw AT commands (`AT`,
`AT+CSQ`, `AT+CREG?`, ...). Raw NMEA sentences from the GPS are echoed to
Serial too, so you can confirm it has a fix independent of the app logic.

## Build

```
pio run                # build firmware
pio run -t upload      # flash firmware
pio run -t uploadfs    # flash the dashboard page (LittleFS)
pio device monitor     # serial console
```

## Known gaps

- Fault trip latches until power-cycle/reset — no remote or automatic reset
- No auth on the AP dashboard
- No fault history persisted across reboot
- ESP32 ADC is nonlinear near the rails; the RMS sampling here is good enough for threshold tripping, not for calibrated metering
