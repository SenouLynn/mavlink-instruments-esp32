# Wiring diagram: ESP32-S3, ST7789, and ArduPilot

The design uses three independent interfaces:

1. **USB-C between the Mac and ESP32-S3** for firmware upload and the serial monitor.
2. **SPI between ESP32-S3 and ST7789** for pixels.
3. **Receive-only UART between the flight controller and ESP32-S3** for MAVLink telemetry.

Do not connect the flight controller's USB-C port to the ESP32 USB-C port. Neither board is
configured here as a USB host, and USB is unnecessary for this telemetry path.

## Complete connection diagram

```text
                         DEVELOPMENT ONLY
  Mac USB-C  <=================================>  ESP32-S3 USB port
                    flash + serial monitor

  ST7789 DISPLAY                                  ESP32-S3 SUPERMINI
  ┌──────────────┐                                ┌──────────────────────┐
  │ VCC          ├───────────────────────────────►│ 3V3                  │
  │ GND          ├───────────────────────────────►│ GND                  │
  │ SCL/SCK/CLK  ├───────────────────────────────►│ GPIO12  SPI clock    │
  │ SDA/MOSI/DIN ├───────────────────────────────►│ GPIO11  SPI data     │
  │ CS           ├───────────────────────────────►│ GPIO10  chip select  │
  │ DC/A0        ├───────────────────────────────►│ GPIO9   data/command │
  │ RES/RST      ├───────────────────────────────►│ GPIO8   reset        │
  │ BL/LED       ├───────────────────────────────►│ 3V3 (*)              │
  └──────────────┘                                │                      │
                                                  │ GPIO18  UART RX      │◄──┐
                                                  │ GND                  │◄┐ │
                                                  └──────────────────────┘ │ │
                                                                           │ │
  ARDUPILOT FLIGHT CONTROLLER — TELEM1 OR TELEM2                           │ │
  ┌──────────────────────────────────────────────────────────────────────┐ │ │
  │ GND  ────────────────────────────────────────────────────────────────┘ │
  │ TX   ─────────────────────────────────────────────────────────────────┘
  │ RX   ───── NOT CONNECTED
  │ CTS  ───── NOT CONNECTED
  │ RTS  ───── NOT CONNECTED
  │ 5V   ───── NOT CONNECTED
  └────────────────────────────────────────────────────────────────────────┘

  (*) Only if the display breakout specifies a 3.3 V-compatible, current-limited
      BL input. Do not power a raw backlight LED without its specified driver/resistor.
```

## ESP32-S3 to ST7789 pin table

This assignment matches the firmware and uses the ESP32-S3 SPI2/FSPI native SCLK, MOSI, and CS
signals. It is intentionally different from the article's working GPIO13/12 mapping; the GPIO
matrix permits both, while GPIO12/11/10 provides the native SPI2 route. Match GPIO numbers printed
on the SuperMini rather than positions copied from a DevKitC diagram.

| ST7789 marking | Meaning | SuperMini label |
|---|---|---:|
| `VCC` | Display logic/panel power | `3V3` |
| `GND` | Ground | `GND` |
| `SCL`, `SCK`, or `CLK` | SPI clock | `GPIO12` |
| `SDA`, `MOSI`, or `DIN` | SPI data from ESP32 | `GPIO11` |
| `CS` | Chip select | `GPIO10` |
| `DC` or `A0` | Data/command select | `GPIO9` |
| `RES` or `RST` | Display reset | `GPIO8` |
| `BL`, `BLK`, or `LED` | Backlight | `3V3`* |

On these SPI displays, a pin marked `SDA` usually means serial data/MOSI; it is not an I2C bus.
There is no MISO connection because the firmware does not read pixels from the panel.

Before connecting VCC or BL, check the display PCB/listing for its accepted supply voltage and
backlight current limiting. This table assumes a breakout module designed for 3.3 V operation,
not a bare LCD panel.

## Flight controller to ESP32 UART

Use a hardware telemetry/UART connector—normally `TELEM1` or `TELEM2`—not USB-C. UART wiring is
crossed by function: the flight controller transmits and the ESP32 receives.

| Flight controller signal | ESP32-S3 | Connection |
|---|---|---|
| `TX` / TX OUT | `GPIO18` / UART1 RX | Connect |
| `GND` | `G` / GND | Connect |
| `RX` / RX IN | — | Leave disconnected |
| `CTS` | — | Leave disconnected |
| `RTS` | — | Leave disconnected |
| Telemetry-port `5V` | — | Leave disconnected |

GPIO18 is exposed on the SuperMini header. The ESP32-S3 GPIO matrix can route UART1 RX to it, and
the code opens UART1 as RX-only with TX set to `-1`.

For a standard six-pin Pixhawk TELEM connector, the conventional assignments are:

| TELEM connector pin | Signal | Use here |
|---:|---|---|
| 1 | +5 V | Do not connect |
| 2 | TX OUT, 3.3 V UART | Connect to ESP32 GPIO18 |
| 3 | RX IN, 3.3 V UART | Do not connect |
| 4 | CTS | Do not connect |
| 5 | RTS | Do not connect |
| 6 | GND | Connect to ESP32 GND |

That pin order is standardized on many Pixhawk-family controllers, but connector families and
pin orders do vary. Verify the exact flight-controller model's pinout before inserting or
repinning a cable. ArduPilot documents the standard layout as TX on pin 2 and GND on pin 6:
[Pixhawk TELEM pinout](https://ardupilot.org/plane/docs/common-pixhawk-overview.html#serial-1-telem-1-and-serial-2-telem-2-pins).

Keeping the flight controller's RX disconnected is a physical enforcement of the read-only
boundary. The ESP32 cannot send commands, stream requests, or heartbeats to ArduPilot.

## Firmware definitions

All board-specific values live in `include/hardware_config.hpp`:

```cpp
#define HUD_TFT_NATIVE_WIDTH 240
#define HUD_TFT_NATIVE_HEIGHT 320
#define HUD_TFT_ROTATION 0

#define HUD_TFT_SCLK_PIN 12
#define HUD_TFT_MOSI_PIN 11
#define HUD_TFT_CS_PIN 10
#define HUD_TFT_DC_PIN 9
#define HUD_TFT_RST_PIN 8
#define HUD_TFT_BL_PIN -1

#define HUD_MAVLINK_RX_PIN 18
#define HUD_MAVLINK_BAUD 115200U
```

`HUD_TFT_BL_PIN = -1` means firmware does not drive the backlight. If the exact display exposes
a logic-level backlight-enable pin—not a raw LED supply—it can later be assigned a spare GPIO
and controlled in software.

## ArduPilot configuration

For the chosen telemetry connector, configure the corresponding logical serial port:

```text
SERIALx_PROTOCOL = 2     # MAVLink 2
SERIALx_BAUD     = 115   # ArduPilot's value for 115200 baud
```

For many controllers, TELEM1 corresponds to `SERIAL1` and TELEM2 to `SERIAL2`, but verify the
controller documentation. ArduPilot's serial-port mapping is logical and can vary by board:
[telemetry port setup](https://ardupilot.org/copter/docs/common-telemetry-port-setup.html).

The flight controller must proactively emit `ATTITUDE`, `VFR_HUD`, `GLOBAL_POSITION_INT`, and
`GPS_RAW_INT`, because this physically receive-only HUD cannot request message intervals.

## Power during staged bring-up

1. First power only the ESP32 and display from the ESP32 USB connection. Leave the flight
   controller disconnected.
2. Confirm demo mode renders correctly.
3. Power everything off.
4. Add only flight-controller TX and GND to the ESP32.
5. Power the flight controller from its normal power module/BEC and the ESP32 from its USB or a
   separately verified regulator.

Do not join two unrelated 5 V supply outputs. The UART requires a shared ground, not a shared
5 V rail.

## Installed power

USB from the development Mac is not an installed power architecture. For vehicle installation,
use a regulator/BEC whose input range covers the aircraft supply and whose continuous and transient
current ratings cover the ESP32-S3, display logic, and backlight together. Connect its regulated
output to the SuperMini input specified by that board vendor, never directly to `3V3` unless it is
a regulated 3.3 V rail designed for that connection. Retain the common signal ground with the
flight controller.

Do not assume a TELEM connector can power the assembly. If its 5 V output is deliberately used,
verify the exact flight-controller output rating and measure worst-case startup and backlight
current first. Otherwise leave TELEM 5 V disconnected as shown above.
