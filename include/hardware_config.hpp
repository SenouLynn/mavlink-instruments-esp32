#pragma once

// Change only this file when the exact ESP32-S3 board or display breakout differs.
// Defaults target a native 240x320 ST7789 panel in portrait orientation.
#define HUD_TFT_NATIVE_WIDTH 240
#define HUD_TFT_NATIVE_HEIGHT 320
#define HUD_TFT_ROTATION 0

// ESP32-S3 SuperMini wiring using the SPI2/FSPI native SCLK, MOSI, and CS pins.
#define HUD_TFT_SCLK_PIN 12
#define HUD_TFT_MOSI_PIN 11
#define HUD_TFT_CS_PIN 10
#define HUD_TFT_DC_PIN 9
#define HUD_TFT_RST_PIN 8
// Backlight is wired to 3V3 for the simplest bring-up. Set this to a GPIO only
// after confirming the display breakout exposes a logic-level BL enable input.
#define HUD_TFT_BL_PIN -1
#define HUD_TFT_BL_ACTIVE_HIGH 1
#define HUD_TFT_SPI_HZ 40000000U

// Receive-only MAVLink: flight-controller TX -> this RX pin. Leave ESP TX disconnected.
#define HUD_MAVLINK_RX_PIN 18
#define HUD_MAVLINK_BAUD 115200U
#define HUD_MAVLINK_RX_BUFFER_BYTES 1024U

#define HUD_FRAME_RATE_HZ 20U
#define HUD_TELEMETRY_STALE_MS 1500U
#define HUD_USE_FRAMEBUFFER 1
