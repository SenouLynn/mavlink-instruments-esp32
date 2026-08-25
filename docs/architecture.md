# Port architecture

## Data flow

```text
ArduPilot TELEM TX
       |
       v  receive-only UART bytes
MAVLink parser (src/main.cpp)
       |
       v  latest typed messages + timestamps
TelemetrySample
       |
       +--> heading/scalar/track resolvers
       +--> predictive trajectory integrator
       |
       v
compose_unified_hud (portable draw commands)
       |
       +--> GFX framebuffer --> ST7789 SPI panel
       `--> SVG target ------> desktop preview
```

`hud-core` includes no Arduino, MAVLink, display-driver, networking, or dynamic-allocation API.
It can therefore move to another microcontroller, renderer, simulator, or test harness without
changing its behavior.

## Source-of-truth mapping

The port follows these files in `~/Desktop/flight-path-hud/packages/hud-ui/src`:

| Browser source | Firmware implementation |
|---|---|
| `logic/telemetry.ts` | `hud/telemetry.hpp` and the MAVLink adapter |
| `logic/heading.ts` | `resolve_heading` |
| `logic/attitude.ts` | Euler body-rate transforms in `logic.cpp` |
| `logic/flightPath.ts` | scalar and ground-track resolvers |
| `logic/trajectory.ts` | fixed-capacity trajectory integrator |
| `components/HudUnifiedInstrument.tsx` | `compose_unified_hud` |

The C++ port uses a fixed 41-point trajectory buffer instead of JavaScript arrays. At the
default 0.25-second step over five seconds, 21 points are used. There is no per-frame heap
allocation in the core.

## Rendering choices

The browser's translucent SVG surfaces are represented by solid RGB565 line colors because
the ST7789 has no alpha blending. The geometry and cue meanings are preserved: world grid and
pitch ladder counter-roll, the predicted corridor rolls with the flight path, cyan/orange path
segments show climb/descent, the vario and airframe stay screen-fixed, and abnormal stall or
coordination states are called out.

The firmware tries to allocate a 240x320 RGB565 framebuffer (153,600 bytes). This removes
flicker. If allocation fails on a low-memory board, it automatically renders directly to the
panel. Set `HUD_USE_FRAMEBUFFER` to `0` if required.

## Telemetry precedence and failure behavior

- Heading: `VFR_HUD.heading`, then `ATTITUDE.yaw`, then `GLOBAL_POSITION_INT.hdg`.
- Ground speed: `VFR_HUD.groundspeed`, then `GLOBAL_POSITION_INT.vx/vy`, then `GPS_RAW_INT.vel`.
- Airspeed: `VFR_HUD.airspeed`; ground speed is the explicit fallback.
- Climb: `VFR_HUD.climb`, then inverted NED `GLOBAL_POSITION_INT.vz`.
- Track: NED `GLOBAL_POSITION_INT.vx/vy`, then `GPS_RAW_INT.cog/vel`.

Message groups older than 1.5 seconds are removed from the render snapshot. A stale link shows
`NO TELEMETRY`; a live link without fresh attitude shows `ATTITUDE STALE`. Missing values use
safe zero/fallback behavior and never leave the device.
