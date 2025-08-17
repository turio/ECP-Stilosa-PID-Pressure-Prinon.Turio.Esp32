# Delonghi ESP32 Temperature Pressure Control & Monitor
IOT Project for acheiving PID like temperature control and pressure profiling on Delonghi ECP/ Stilosa Espresso machines without any drilling or difficult mods that require manual labour

ESP32-based controller code for monitoring and controlling temperature and pressure on Delonghi espresso machines such as ECP series and Stilosa. 

This project reads a K-type thermocouple (MAX6675) and a pressure transducer, controls a heater relay, and offers a small web UI, OTA updates and an optional OLED status display.


## Features
- Reads temperature from a K-type thermocouple via the MAX6675 amplifier.
- Reads pressure from an analog pressure sensor (ADC1 pin).
- Controls a heater via a relay (safety limits and early-cutoff logic included).
- Temperature smoothing (EMA) and calibration support.
- Presumed-off/standby detection when machine cools below a threshold.
- Simple web server for live stats, shot timer and plots.
- OTA support (Arduino OTA) and mDNS support.
- Optional SSD1306 OLED status output.

## Hardware / BOM (short)
See `bom.md` for a fuller list. Key parts:
- ESP32 dev board
- MAX6675 module + K-type thermocouple
- Pressure sensor (0–1.6 MPa or similar mapped to 0–16 bar in code)
- SSR or mechanical relay module to switch heater
- SSD1306 OLED (optional)

## Pinout (as used in `src/main.cpp`)
- MAX6675 SCK -> GPIO18
- MAX6675 CS  -> GPIO5
- MAX6675 SO  -> GPIO19 (MISO)
- Pressure sensor analog -> GPIO35 (ADC1_CH7)
- Relay control -> GPIO14
- Status LED -> GPIO27

Adjust physical wiring to match your board and the pin defines in `src/main.cpp`.

## Important configuration
- Wi-Fi SSID / password: edit `src/main.cpp` (constants `ssid` and `password`) before first flash.
- PlatformIO upload: `platformio.ini` currently configures `upload_protocol = espota` and an `upload_port` IP — change this to your device IP for OTA upload, or switch to USB by removing `upload_protocol` / `upload_port`.

## Build & Flash
This project uses PlatformIO (see `platformio.ini`). Typical steps:

1. Open the folder in VS Code with the PlatformIO extension.
2. Install project libraries (PlatformIO will auto-install via `lib_deps`).
3. Configure Wi‑Fi credentials in `src/main.cpp` and set the correct `upload_port` in `platformio.ini` if using OTA.
4. Build and upload from the PlatformIO toolbar, or use a USB serial upload by switching upload settings.

## Calibration & Tuning
- Temperature: `src/main.cpp` contains arrays for calibration points (`raw_temps_c` and `actual_temps_c`) and a `getCalibratedTemperature()` function. Add or adjust points for better accuracy.
- Pressure: The code contains calibration constants `VOLTS_AT_0_BAR` and `VOLTS_AT_16_BAR` and converts ADC readings to bar. Update them to match your sensor's output curve.
- Smoothing: EMA alpha and ADC smoothing buffer sizes are exposed as constants in `main.cpp`.

## Safety notes
- There are safety limits in code (maximum heater on duration, early-cutoff, and cooldown timers) but verify operation thoroughly before connecting to mains or switching high current loads.
- Use a suitably rated SSR or mechanical relay with proper isolation, fusing and wiring practices.

## Troubleshooting
- ADC2 vs ADC1: `pressureSensorPin` uses GPIO35 (ADC1) so it works reliably while Wi‑Fi is active. If you change pins, prefer ADC1 pins.
- If OTA upload fails, revert to USB upload or ensure the `upload_port` IP matches the device and that ArduinoOTA is running on the device.
- If temperature reads as NaN or unstable, check thermocouple wiring and the MAX6675 module power/GND.

## License
This project is released under the MIT License. You are free to use, modify and redistribute the code and documentation for personal or commercial purposes.

Conditions: please retain the copyright and license notice and give credit to the original author: Prinon Turio

See the bundled `LICENSE` file for the full license text.


