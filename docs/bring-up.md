# ESP32-S3 + ST7789 bring-up

Work through these stages in order. Do the display-only demo before connecting the flight
controller.

## 1. Identify the exact boards

Confirm the processor marking is `ESP32-S3FH4R2`; that is the 4 MB flash/2 MB QSPI-PSRAM target
described by `boards/esp32-s3-supermini.json`. Stop and create a matching board definition if the
marking differs. Write down every label on the display connector. A typical SPI
display exposes `GND`, `VCC`, `SCL/SCK/CLK`, `SDA/MOSI/DIN`, `RES/RST`, `DC`, `CS`, and `BL/LED`.
`SDA` on these modules often means SPI MOSI, not I2C SDA. This firmware does not use MISO.

The confirmed display geometry is a rectangular 240x320 portrait canvas. The firmware
initializes the ST7789 at its native 240x320 dimensions with rotation `0`.

## 2. Wire only the display

Power everything off and unplug USB before changing wiring.

| Display label | ESP32-S3 default | Firmware name |
|---|---:|---|
| GND | GND | — |
| VCC | 3V3 only unless the module explicitly documents 5 V input | — |
| SCL / SCK / CLK | GPIO 12 | `HUD_TFT_SCLK_PIN` |
| SDA / MOSI / DIN | GPIO 11 | `HUD_TFT_MOSI_PIN` |
| CS | GPIO 10 | `HUD_TFT_CS_PIN` |
| DC / A0 | GPIO 9 | `HUD_TFT_DC_PIN` |
| RES / RST | GPIO 8 | `HUD_TFT_RST_PIN` |
| BL / LED | 3V3 for initial bring-up | `HUD_TFT_BL_PIN = -1` |

These are locked assignments for the ESP32-S3 SuperMini. Match printed GPIO
labels, not a wire's position in a photograph. The initial setup keeps the backlight off a GPIO:
connect BL to 3V3 only when the breakout documentation confirms that BL accepts 3.3 V and
includes the necessary current limiting. A raw LED backlight requires its specified resistor or
driver. See [the complete wiring diagram](wiring.md).

## 3. Build and flash demo mode

The easiest workflow is VS Code with the PlatformIO IDE extension:

1. Open this repository as the folder.
2. Wait for PlatformIO to install the ESP32 toolchain and libraries.
3. Run **PlatformIO: Build**.
4. Connect the ESP32-S3 over USB and run **PlatformIO: Upload and Monitor**.

CLI equivalent:

```bash
pio run
pio run --target upload
pio device monitor --baud 115200
```

Expected monitor output begins with `Unified HUD booting`, reports a 240x320 display, approximately
2 MB total PSRAM, and `framebuffer=yes`. The screen should immediately animate in demo mode.
`PSRAM/FRAMEBUFFER STARTUP FAILURE` is a hard configuration or memory fault, not a supported
low-performance mode.

If upload cannot find the board, hold **BOOT**, tap **RESET**, start upload, and release **BOOT**
when writing begins. Exact bootloader behavior varies by board.

## 4. Validate the display before MAVLink

Confirm all of the following:

- black background, blue perspective grid, and orange fixed aircraft symbol;
- smooth banking and pitching with no full-screen flashing;
- the whole 240x320 image is visible in portrait, with the vario at the right edge;
- text is upright and colors are plausible;
- serial output continues without resets and free heap remains stable.

Common corrections are isolated in `include/hardware_config.hpp`:

- blank but lit: recheck CS/DC/RST and SPI pins;
- completely dark: check BL polarity/power and VCC/GND;
- portrait or upside down: try rotations `0` through `3`;
- cropped or offset: correct native width/height for the exact panel variant;
- random pixels/resets: reduce `HUD_TFT_SPI_HZ` from 40 MHz to 20 MHz and shorten wires;
- `PSRAM/FRAMEBUFFER STARTUP FAILURE`: verify the FH4R2 marking and QSPI-PSRAM board definition.

## 5. Add receive-only flight-controller telemetry

With both devices powered off:

| Flight controller telemetry port | ESP32-S3 |
|---|---|
| TX | GPIO 18 (`HUD_MAVLINK_RX_PIN`) |
| GND | GND |
| RX | **leave disconnected** |
| 5 V | **leave disconnected unless a deliberate, verified power design requires it** |

The crossed UART connection is intentional: flight-controller TX feeds ESP32 RX. Connecting
grounds is required. Verify the flight controller exposes 3.3 V UART logic. Do not connect an
inverted SBUS pin or an RS-232-level port.

In Mission Planner's full parameter list, configure the chosen physical telemetry port's
logical `SERIALx` entry for MAVLink 2 (`SERIALx_PROTOCOL = 2`) and 115200 baud
(`SERIALx_BAUD = 115` on standard ArduPilot parameter enumerations), then reboot the controller.
The physical connector-to-`SERIALx` mapping is board-specific; use the controller's ArduPilot
hardware page rather than guessing.

The HUD needs these outgoing messages:

| MAVLink message | ID | Useful bench rate |
|---|---:|---:|
| `ATTITUDE` | 30 | 20 Hz |
| `VFR_HUD` | 74 | 10 Hz |
| `GLOBAL_POSITION_INT` | 33 | 10 Hz |
| `GPS_RAW_INT` | 24 | 2 Hz |

For the first bench test, leaving a GCS attached to the same outgoing telemetry stream is often
the simplest way to establish rates. For a standalone setup, configure persistent ArduPilot
stream rates or a `message-intervals-chanN.txt` file on the flight controller SD card. ArduPilot
documents the exact method in its [telemetry port setup](https://ardupilot.org/copter/docs/common-telemetry-port-setup.html)
and [message interval guide](https://ardupilot.org/dev/docs/mavlink-requesting-data.html). The
`chanN` number is the order of MAVLink-enabled serial ports, not necessarily the `SERIALx`
suffix.

## 6. Prove the live link on the bench

Power the propeller-free vehicle and watch the USB serial monitor. The first valid MAVLink frame
changes `mode=demo` to `mode=live`. Every two seconds the monitor reports:

```text
mode=live sysid=1 mavlink_messages=... parse_errors=... uart_errors=0 age_ms=...
heap=... largest=... psram_free=... render_max_us=...
```

Acceptance checks:

- `mavlink_messages` rises continuously;
- `parse_errors` stays at zero or does not continually rise;
- `uart_errors` remains zero under sustained traffic;
- `render_max_us` remains below the 50,000 us frame period at 20 Hz;
- `age_ms` stays well below 1500;
- tilting the unarmed controller moves pitch/roll in the expected direction;
- rotating it changes the heading readout;
- stopping/disconnecting telemetry produces `NO TELEMETRY` within 1.5 seconds;
- reconnecting restores the scene without an ESP32 reboot.

Remove propellers for every bench test. The HUD is read-only, but powered flight hardware still
requires normal restraint and battery safety.
