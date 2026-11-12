# Light Painting Arm

Turns a 3D model or an image into paths, previews them like a slicer, and drives the
3-joint arm (base, shoulder, elbow) to draw them in the air with an RGB LED while a
camera takes a long exposure.

## Run it

    ./run.sh

This opens the viewer at http://127.0.0.1:8765. It starts on a built-in simulator, so
everything works with nothing plugged in.

1. **Source** – drop an STL / OBJ / PLY / GLB model or a PNG / JPG image (or click an example).
   Models are drawn as their sharp edges; images are traced as edges, outlines or centre lines.
   An OBJ that contains line elements (`l 1 2 3 …`) is drawn exactly as written, one stroke
   per line, which is the way to hand-author a drawing (see `examples/make_examples.py`).
2. **Placement** – *Auto-fit* finds the largest size the arm can reach. Anything shown in
   red is out of reach and will be skipped.
3. **Light** – colours, draw speed and brightness. *Even exposure* dims the LED wherever the
   arm slows down, so corners and stroke ends don't burn in as bright dots.
4. **Arm** – pick the ESP32's serial port, *Connect*, *Home*, then *Draw*. *Hold* is how long
   the arm waits at the start point so you can open the shutter. "Exposure needed" in the
   plan is how long the shutter has to stay open.

   *Animation* turns the drawing a little further for each frame, for stop-motion: set the
   number of frames, the turn per frame and the axis, scrub *Preview frame* to check each
   one, and *Auto-fit* sizes the drawing so every frame is reachable. *Draw all frames* draws
   them in turn, one long exposure each; between frames it waits for *Next frame* (or a set
   number of seconds) while you close the shutter and re-open it.
5. **Speed** – a live override from 10% to 200% of the planned speed. It scales drawing,
   travel and jogging, works mid-drawing (it takes about a second to take effect), and stays
   set between runs. Above 100% the arm exceeds the limits in `arm_config.json`.

The timeline at the bottom scrubs through the plan (blue ghost arm). The solid arm is always
the real arm's reported position, and while it draws, the lines light up as it finishes them.

## Before the first real run

**1. Measure the arm** and edit `arm_config.json` (the numbers in there are estimates from
your CAD screenshot). Restart the program after editing.

| Setting | Meaning |
|---|---|
| `shoulder_height` | table to the shoulder axis |
| `upper_arm` | shoulder axis to elbow axis |
| `forearm` | elbow axis to the LED |
| `shoulder_forward` | how far the shoulder axis sits in front of the base axis (0 if it crosses it) |
| `lateral_offset` | sideways distance from the base axis to the line the LED swings in |
| `min_tip_height` | lowest height the LED may go |
| `home_deg` | each joint's angle when its hall sensor triggers |
| `min_deg` / `max_deg` | joint limits; the planner never leaves them |
| `direction` | `1` or `-1`; flip it if a joint turns the wrong way |
| `max_speed_dps`, `max_accel_dps2`, `max_jump_dps` | speed, acceleration and the instant speed change a joint can take without ramping |

Angles: base 0° points along +X (forward) and is positive counter-clockwise seen from above;
shoulder is the upper arm's angle above horizontal (90° = straight up); elbow is the forearm
relative to the upper arm (0° = straight, negative = folded down).

**2. Flash the firmware.** Open `firmware/` in PlatformIO, set the pins and homing
directions in `firmware/src/config.h`, and upload. Joint 0 defaults to the pins from your
homing test (STEP 25, DIR 26, EN 27, hall 32); the pins for the other two joints and the
LED are placeholders.

**3. Check each joint with the arm clear of everything:**

- Home it, either way described under "Homing" below.
- Jog each joint by +10°. If the real joint moves opposite to the one on screen, flip
  `direction` in `arm_config.json`.
- Run the octahedron example small and slow first. The speed limits start low (20°/s);
  raise them once the motors run cleanly.

## Homing

**At marks (uses the encoders).** Scribe a mark across each joint at its home position.

1. Once: put every joint *exactly* on its mark (jog with the ±1° buttons, which work before
   homing, or press *Release motors* and turn the joints by hand), then click *save the
   marks*. The ESP32 stores each encoder's angle at that pose in flash.
2. Every power-up: get each joint to within a few degrees of its mark and press
   *Home at marks*. The encoder gives the motor's exact angle within its current turn, and
   one motor turn is only 12.4° at the joint, so the firmware can tell exactly how far off
   each joint is. Lining up by eye only has to be good to about ±6°. The log line
   "homed at marks; joints were off by …" shows what it corrected.

A joint without an encoder is simply taken to be exactly on its mark.

**With hall sensors.** *Home (sensors)* makes each joint seek its sensor, back off, and creep
back onto it (same routine as your homing test). If a joint heads the wrong way, flip its
`HOME_DIR` in `config.h`.

## Missed steps

The firmware reads each motor's MT6701 and compares it with the step count. The *Missed
steps* row shows the difference per joint, in degrees at the joint. It stays near zero and
turns red past `slip_warn_deg` (0.15°; a stalled stepper falls back in jumps of about 0.25°).

- During a drawing, the log records which joint slipped and when, and the finish message
  lists the worst error. If that happens, lower that joint's `max_speed_dps` or
  `max_accel_dps2` and try again.
- *Resync* makes the arm's position follow the encoders, so the next move starts from where
  the arm really is. Every jog, *Go to home pose* and *Draw* does this automatically first.
- With the motors released, joints that have encoders are tracked as you turn them by hand.

Encoder wiring: the MT6701s all share one I2C address, so each needs its own SDA pin; SCL
can be shared (`PIN_ENC_SDA` / `PIN_ENC_SCL` in `config.h`, -1 for no encoder). The firmware
prints "encoder N ok / NOT FOUND" at boot. If the missed-steps reading runs away as soon as
a joint moves, that encoder counts backwards: flip its `ENC_DIR`.

Slips are detected and corrected between moves, not during one: a line being drawn when a
motor slips will still be drawn in the wrong place.

## How it fits together

    lightpaint/mesh_paths.py    model -> sharp edges -> long strokes
    lightpaint/image_paths.py   image -> traced lines on a flat plane
    lightpaint/kinematics.py    joint angles <-> LED position
    lightpaint/planner.py       strokes -> timed joint samples (speed/accel limits, LED levels)
    lightpaint/device.py        serial link to the ESP32, and the simulator
    lightpaint/server.py        web server, streams the plan to the arm
    web/                        the 3D viewer (three.js)
    firmware/                   ESP32 segment player (PlatformIO)

The PC does all the planning and streams the motion as 20 ms segments ("be at these step
counts, LED this colour"). The ESP32 spreads each segment's steps evenly across all three
motors from a 40 kHz timer, so the joints stay in sync. The serial protocol is documented at
the top of `firmware/src/main.cpp`.

Tests: `.venv/bin/python tests/test_core.py`
