# UI and firmware iteration loop

## Fast loop: no hardware

```bash
make test
make preview
open build/hud-preview.svg
```

The SVG is composed by the same `compose_unified_hud` function used on the ESP32. Change scene
geometry in `lib/hud-core/src/scene.cpp`; change telemetry or trajectory behavior in
`lib/hud-core/src/logic.cpp`; add a corresponding assertion in `test/hud_tests.cpp`.

Use `native/hud_preview.cpp` as the deterministic telemetry fixture. Keeping a few named fixture
states there (level, banked turn, climb, stall, stale) is the next useful extension once the
first real-panel photos establish color and optical constraints.

## Panel loop

1. Run native tests and inspect the SVG.
2. Run `pio run` before connecting the board.
3. Upload and watch the serial monitor through at least ten report lines.
4. Photograph the panel straight-on in a dark room and, separately, through the beam splitter.
5. Record: firmware commit, display rotation, SPI speed, framebuffer status, and observed FPS.

Change one visual variable at a time. The most useful tuning constants are concentrated at the
top of `scene.cpp`: projection focal distance, near distance, camera height, corridor half-width,
pitch pixels per degree, and ladder spacing. Flight behavior constants are in `TrajectoryConfig`.

## Logic parity loop

When the browser spike changes, compare its TypeScript test vector against `hud_tests.cpp` and
port behavior before styling. Important invariants are:

- NED velocity signs: north `vx`, east `vy`, climb is negative `vz`;
- angular rates are body rates and require the Euler transform;
- airspeed controls stall and forward reach, with groundspeed only as fallback;
- explicit zero body rates override bank-derived turn rate;
- missing rates allow the coordinated-turn fallback;
- no NaN/Inf or unbounded trajectory point reaches the renderer.

## Evidence to attach to an issue

Include the exact board model, a link/photo of the display PCB, wiring table, serial monitor
excerpt, whether demo mode works, whether live message count rises, a screen photo, and the
smallest config/code change that reproduces the issue. That separates display, transport,
telemetry-rate, math, and optical problems quickly.

