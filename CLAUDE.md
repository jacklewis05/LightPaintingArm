# Light Painting Arm

Software for Jack's 3D-printed 3-joint robot arm (base, shoulder, elbow), which draws
models and images in the air with an RGB LED during a camera long exposure. README.md is
the user-facing guide (setup, calibration, homing, missed steps); read it for behaviour.

## Hardware

- Each joint: NEMA 17 → A4988 at 1/16 microstepping → 3D-printed 29:1 cycloidal drive.
  257.8 microsteps per joint degree; one motor turn = 12.4° at the joint.
- MT6701 magnetic encoder on each **motor** shaft (I2C, fixed address 0x06), hall sensor +
  magnet on each joint output, RGB LED on three PWM pins, classic ESP32 devkit.
- Jack's original single-joint test firmware is at `~/Desktop/EncoderHoming/main (1).cpp`;
  joint 0's pins (STEP 25, DIR 26, EN 27, hall 32, encoder 21/22) come from it.
- Background on the arm: github.com/jacklewis05/jack-lewis-engineering, `src/content.js`,
  the `robotic-arm` entry.

## Commands

    ./run.sh                              # start the app at http://127.0.0.1:8765 (creates .venv on first run)
    .venv/bin/python tests/test_core.py   # tests (plain script; pytest is not installed)
    cd firmware && ~/.platformio/penv/bin/pio run    # compile firmware (pio is not on PATH)

Python is the system 3.9: no `X | Y` type syntax, no `match`. There is no Node; the viewer
is plain ES modules with three.js r160 vendored in `web/vendor/`.

## Architecture

The PC plans everything; the ESP32 is a dumb segment player.

    lightpaint/mesh_paths.py    model -> sharp/boundary edges -> chained strokes
    lightpaint/image_paths.py   image -> traced lines (edges / outline / centerline)
    lightpaint/paths.py         Stroke type, normalise, place, colour
    lightpaint/kinematics.py    FK / IK (vectorised)
    lightpaint/planner.py       strokes -> Job: joint samples every 20 ms + LED colour
    lightpaint/device.py        SerialDevice (ESP32) and SimDevice, same interface
    lightpaint/server.py        FastAPI: plan endpoints, streaming loop, 25 Hz websocket status
    web/app.js                  three.js viewer and all UI logic
    firmware/src/main.cpp       step ISR, homing, encoders; protocol documented at the top
    firmware/src/config.h       pins and homing settings
    arm_config.json             geometry, joint limits, speed limits (PC side only)

- World frame: X forward, Y left, Z up, origin on the table under the base axis. Joint
  angle conventions are in the `lightpaint/config.py` docstring.
- The firmware only knows motor microsteps; step 0 is each joint's `home_deg`. All geometry
  and limits live on the PC.
- A `Job` is one sample per 20 ms: joint angles to reach and the LED colour for that
  interval. `planner.draw` enforces per-joint speed, acceleration and instant-jump limits;
  LED brightness is scaled by tip speed ("even exposure").
- `server.stream` keeps the firmware's 64-segment queue topped up using the `done` counter
  from status lines. Segments carry a sequence number; the firmware rejects gaps.
- An OBJ with `l` line elements bypasses edge detection: each element is one stroke, drawn
  as written (`load_obj_lines`). `examples/make_examples.py` generates `rotunda.obj` this
  way, and `uva_logo.png`; edit the generator, not the outputs.
- Animation = the same drawing with an extra spin about a world axis per frame
  (`paths.place` spin_axis / spin_deg). `server.animate` plans and draws frames one at a
  time; the viewer follows via `run.frame` in the status and `GET /api/job`.
- The live speed control (`S.speed`, 0.1–2.0) only changes each segment's duration in
  `server.stream`; the plan itself is untouched. Above 1.0 it exceeds the config limits.
- Encoder error ("slip") = encoder position − step count. It is reported and corrected
  between moves (`trust_encoders`), never during one.

## Things that must stay in sync

- `TICK_HZ` and `QUEUE_SIZE` in `firmware/src/config.h` ↔ `FIRMWARE_TICK_HZ` in
  `lightpaint/config.py` and `QUEUE_SIZE` in `lightpaint/device.py`.
- The status line format: `sendStatus()` in `main.cpp` ↔ `SerialDevice._handle` (expects
  exactly 16 fields) ↔ `SimDevice.status` ↔ the websocket message in `server.ws` ↔
  `onStatus` in `web/app.js`. A new device capability needs all of these plus both device
  classes.
- Any new command: firmware `handleLine`, the protocol comment at the top of `main.cpp`,
  and a method on both `SerialDevice` and `SimDevice`.

## Gotchas

- Values sent over the websocket must be plain Python types. A numpy bool or float in the
  status message kills the socket silently and the viewer shows "not connected".
- Stopping a background server: `pkill -f "python -m lightpaint"` does **not** match (the
  process is the framework `Python`). Use `kill $(lsof -ti tcp:8765 -sTCP:LISTEN)`. A
  leftover server makes a new one fail to bind while the old code keeps answering.
- STEP and DIR pins must be GPIO 0–31: the ISR writes them through `GPIO.out_w1ts`.
- The ISR must not call `ledcWrite` or do I/O; it sets flags that `loop()` applies.
- The encoders are read with bit-banged I2C, one SDA pin each with a shared SCL, because
  they all have the same address. `encUpdate()` has to keep running in every blocking loop
  or the multi-turn count is lost.
- The installed PlatformIO ESP32 platform uses Arduino core 3.x (`timerBegin(freq)`,
  `ledcAttach`); the code keeps `#if ESP_ARDUINO_VERSION_MAJOR >= 3` branches for 2.x.
- The wide preview lines are drawn opaque on purpose: the 2 mm segments overlap, so any
  blending builds up into white blobs.
- Delete `firmware/.pio` after a test build.

## Verifying changes

- Core logic: `tests/test_core.py`.
- Viewer and server: start a server on a spare port (`--port 8799 --no-browser`; Jack may
  have his own copy running on 8765, so never kill that one), then drive headless Chrome over the DevTools
  protocol (`--headless=new --use-angle=swiftshader --enable-unsafe-swiftshader
  --remote-debugging-port=…`) to click, read the DOM, capture screenshots and collect
  console errors. `?example=octahedron.obj` loads an example on open. `POST /api/sim_slip`
  fakes missed steps on the simulator.
- Firmware can only be compile-checked here. Say so when reporting; nothing in `firmware/`
  has been run on the real arm unless Jack reports it.

## Current state

- Never run on hardware as of 2026-11-11. The geometry, limits and home angles in
  `arm_config.json` are estimates from a CAD screenshot, and every pin except joint 0's is
  a placeholder.
- Known limits: Stop halts without ramping down; dark travel moves are straight in joint
  space and not checked for self-collision; a slip mid-stroke is detected but not corrected
  until the next move.
