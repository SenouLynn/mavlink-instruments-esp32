# MAVLink Instruments for ESP32-S3

Read-only ArduPilot telemetry rendered as the **Unified HUD** on an SPI ST7789 display. The
firmware is deliberately split so the flight math and scene can be tested and previewed on a
Mac without an ESP32 or C++ toolchain setup beyond the system compiler.

This repository is the dedicated hardware port of the Unified HUD and experimental trajectory
projection from `~/Desktop/flight-path-hud`. The browser implementation remains the design
reference; `lib/hud-core` is its framework-free C++ port.

## Start here

1. Generate the desktop preview:

   ```bash
   make test
   make preview
   open build/hud-preview.svg
   ```

2. Follow [the wiring diagram](docs/wiring.md), then read [the bring-up runbook](docs/bring-up.md).
   Confirm the exact board labels and display
   breakout pin labels before applying power.
3. Edit only [hardware_config.hpp](include/hardware_config.hpp) for pin or panel differences.
4. Install the VS Code PlatformIO extension, then use **PlatformIO: Upload and Monitor**, or run:

   ```bash
   pio run
   pio run --target upload
   pio device monitor
   ```

The firmware starts with animated demo telemetry. A valid MAVLink frame switches it to live
mode automatically. Sending `d` in the serial monitor toggles demo/live manually.

## Repository map

| Path | Purpose | Expected edit frequency |
|---|---|---|
| `lib/hud-core` | Portable telemetry resolution, trajectory math, and scene | UI/logic iteration |
| `src/main.cpp` | ST7789 and receive-only MAVLink adapters | Rare |
| `include/hardware_config.hpp` | Pins, panel geometry, baud, refresh rate | Hardware setup |
| `native/` | Deterministic SVG renderer using the same scene | Every visual iteration |
| `test/` | Native behavior and scene smoke tests | Every logic iteration |
| `docs/` | Wiring, flashing, validation, and architecture notes | As setup evolves |

## Safety boundary

The ESP32 opens its flight-controller UART with **RX only**. The firmware never constructs or
sends a MAVLink command, heartbeat, or stream request. Leave the ESP32 TX pin disconnected.
The HUD is not a flight-control dependency: stale or absent telemetry becomes an on-screen
warning and cannot change vehicle state.

## Current assumptions

- Generic ESP32-S3 board compatible with PlatformIO's `esp32-s3-devkitc-1` target.
- ST7789V SPI breakout with a 240x320 portrait canvas.
- 3.3 V UART logic and a common ground with the flight controller.
- MAVLink 1 or 2 byte stream at 115200 baud.

The confirmed screen geometry is a rectangular 240x320 portrait canvas. The ST7789 is
initialized at its native 240x320 dimensions with rotation `0`.
