"""Local web server: serves the viewer, plans jobs, and streams them to the arm."""
from __future__ import annotations

import asyncio
import time
from pathlib import Path
from typing import Optional

import numpy as np
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles

from . import planner
from .config import ArmConfig, FIRMWARE_TICK_HZ, ROOT
from .device import QUEUE_SIZE, SerialDevice, SimDevice, list_serial_ports
from .image_paths import IMAGE_TYPES, load_image_strokes, suggest_mode
from .kinematics import fk
from .mesh_paths import MESH_TYPES, load_mesh_strokes
from .paths import colorize, normalize, parse_hex, place

WINDOW = QUEUE_SIZE - 16     # segments allowed in flight; the margin covers status-report lag
SPEED_MIN, SPEED_MAX = 0.1, 2.0      # range of the live speed control
SPEED_SLEW = 0.03            # largest change in speed per segment, so a big jump is ramped in

cfg = ArmConfig.load()
app = FastAPI()


class State:
    def __init__(self):
        self.file_name: Optional[str] = None
        self.file_data: Optional[bytes] = None
        self.kind: Optional[str] = None           # 'mesh' or 'image'
        self.source_key = None
        self.strokes = None                       # normalised
        self.source_size = None
        self.job: Optional[planner.Job] = None
        self.device = None
        self.task: Optional[asyncio.Task] = None
        self.run = {"state": "idle", "label": "", "progress": 0.0, "k": None, "message": ""}
        self.speed = 1.0          # live speed control: 1.0 plays moves exactly as planned
        self.frame = 0            # animation frame that self.job was planned for
        self.next_frame: Optional[asyncio.Event] = None


S = State()
S.device = SimDevice(cfg)      # start on the simulator so the viewer works with nothing plugged in


def fail(status: int, message: str):
    raise HTTPException(status, message)


def busy() -> bool:
    return S.task is not None and not S.task.done()


# ---------------------------------------------------------------- source and planning

def load_source(opts: dict):
    """Parse the uploaded file into normalised strokes (cached on the options that matter)."""
    if S.file_data is None:
        fail(400, "load a model or image first")
    if S.kind == "mesh":
        key = ("mesh", float(opts.get("feature_angle", 20)))
    else:
        key = ("image", opts.get("image_mode", "edges"), float(opts.get("detail", 0.5)),
               bool(opts.get("invert", False)))
    if key != S.source_key:
        ext = S.file_name.rsplit(".", 1)[-1].lower()
        try:
            if S.kind == "mesh":
                strokes = load_mesh_strokes(S.file_data, ext, key[1])
            else:
                strokes = load_image_strokes(S.file_data, key[1], key[2], key[3])
        except ValueError as exc:
            fail(422, str(exc))
        except Exception as exc:
            fail(422, "could not read %s: %s" % (S.file_name, exc))
        S.strokes, S.source_size = normalize(strokes)
        S.source_key = key
    return S.strokes


@app.post("/api/load")
async def api_load(request: Request, name: str):
    ext = name.rsplit(".", 1)[-1].lower() if "." in name else ""
    if ext in MESH_TYPES:
        kind = "mesh"
    elif ext in IMAGE_TYPES:
        kind = "image"
    else:
        fail(415, "unsupported file type .%s (models: %s; images: %s)"
             % (ext, ", ".join(sorted(MESH_TYPES)), ", ".join(sorted(IMAGE_TYPES))))
    S.file_data = await request.body()
    S.file_name, S.kind, S.source_key, S.job = name, kind, None, None
    info = {"name": name, "kind": kind}
    if kind == "image":
        try:
            info["image_mode"] = suggest_mode(S.file_data)
        except ValueError as exc:
            fail(422, str(exc))
    return info


def animation(body: dict):
    """(number of frames, turn axis, degrees per frame) from a request; 1 frame = no animation."""
    anim = body.get("animation", {})
    frames = int(np.clip(int(anim.get("frames", 1)), 1, 360))
    axis = anim.get("axis", "z") if anim.get("axis") in ("x", "y", "z") else "z"
    return frames, axis, float(anim.get("step", 0.0))


def build_job(body: dict, frame: int) -> planner.Job:
    """Plan one frame of the drawing described by a request from the viewer."""
    strokes = load_source(body.get("source", {}))
    frames, axis, step = animation(body)
    pl = {k: float(v) for k, v in body["placement"].items()}
    pl.update(spin_axis=axis, spin_deg=step * frame)
    light = body.get("light", {})
    colored = colorize(strokes, light.get("mode", "palette"), parse_hex(light.get("color", "#ffffff")))
    return planner.plan(
        cfg, place(colored, pl),
        draw_speed=max(1.0, float(light.get("speed", 40))),
        brightness=float(np.clip(light.get("brightness", 1.0), 0.0, 1.0)),
        even_exposure=bool(light.get("even_exposure", True)),
        return_home=bool(light.get("return_home", True)),
    )


def job_json(job: planner.Job) -> dict:
    return {
        "dt": job.dt,
        "tip": np.round(job.tip, 1).ravel().tolist(),
        "q": np.round(job.q, 2).ravel().tolist(),
        "path": job.path.tolist(),
        "stroke_rgb": job.stroke_rgb,
        "unreachable": [np.round(u, 1).ravel().tolist() for u in job.unreachable],
        "stats": job.stats,
        "frame": S.frame,
        "source": {"name": S.file_name, "kind": S.kind, "strokes": len(S.strokes),
                   "size": [round(float(v), 2) for v in S.source_size]},
    }


@app.post("/api/autofit")
def api_autofit(body: dict):
    strokes = load_source(body.get("source", {}))
    pl = body.get("placement", {})
    rot = {k: float(pl.get(k, 0)) for k in ("rx", "ry", "rz")}
    frames, axis, step = animation(body)
    return planner.autofit(cfg, strokes, rot, spins=[(axis, step * i) for i in range(frames)])


@app.post("/api/plan")
def api_plan(body: dict):
    if busy():
        fail(409, "the arm is moving; stop it before changing the plan")
    frame = int(np.clip(int(body.get("frame", 0)), 0, animation(body)[0] - 1))
    S.job, S.frame = build_job(body, frame), frame
    return job_json(S.job)


@app.get("/api/job")
def api_job():
    """The plan currently loaded (during an animation: the frame being drawn)."""
    if S.job is None:
        fail(404, "no plan yet")
    return job_json(S.job)


# ---------------------------------------------------------------- device control

def device():
    if S.device is None:
        fail(409, "not connected: pick the simulator or a serial port first")
    return S.device


def current_q() -> np.ndarray:
    return cfg.steps_to_deg(device().status()["steps"])


def slip_deg(st: dict):
    """Per joint: how far the encoder says the joint is from where the step count puts it,
    in joint degrees (None for a joint with no encoder)."""
    return [float(cfg.direction[j] * st["slip"][j] / cfg.steps_per_deg) if st["enc"][j] else None
            for j in range(3)]


def slipped(st: dict):
    return [bool(d is not None and abs(d) > cfg.slip_warn) for d in slip_deg(st)]


async def trust_encoders():
    """Before planning a move from "where the arm is", make sure that is really where it
    is: if the encoders show missed steps, adopt their position."""
    dev = device()
    if any(slipped(dev.status())):
        dev.resync()
        await asyncio.sleep(0.25)


async def stream(q: np.ndarray, rgb: np.ndarray, label: str, job_from: Optional[int] = None,
                 hold_s: float = 0.0, extra: Optional[dict] = None) -> str:
    """Feed samples to the device, keeping its queue topped up, until every one has run.

    q[0] must be the pose the arm is in now. job_from is the index in q where the planned
    job starts (after the approach move); the viewer uses it to light up finished strokes.
    hold_s pauses at that point with the arm still, e.g. to open the camera shutter.
    extra is added to the run status shown in the viewer. Returns "done", "stopped" or "failed"."""
    dev = S.device
    steps = cfg.deg_to_steps(q)
    total = len(q) - 1
    S.run = dict({"state": "running", "label": label, "progress": 0.0, "k": None, "message": ""},
                 **(extra or {}))
    try:
        dev.new_job()
        await asyncio.sleep(0.2)      # let any status line from before the reset drain
        sent, held, stalled = 0, hold_s <= 0 or job_from is None, time.monotonic()
        last_done = -1
        worst = [0.0, 0.0, 0.0]       # largest encoder disagreement seen on each joint
        # The live speed control stretches or shrinks each segment's duration. `rate`
        # follows S.speed gradually, and `owed` carries the rounding to whole milliseconds.
        rate, owed = S.speed, 0.0
        step_change = np.abs(np.diff(steps, axis=0)).max(axis=1)
        while True:
            st = dev.status()
            for j, d in enumerate(slip_deg(st)):
                if d is not None and abs(d) > abs(worst[j]):
                    if abs(d) > cfg.slip_warn >= abs(worst[j]):
                        dev.log.append("missed steps on %s at %.1f s into %s"
                                       % (cfg.names[j], st["done"] * cfg.dt, label))
                    worst[j] = d
            if dev.error:
                raise RuntimeError(dev.error)
            if not st["online"]:
                raise RuntimeError("lost contact with the arm")
            done = st["done"]
            if done != last_done:
                last_done, stalled = done, time.monotonic()
            elif time.monotonic() - stalled > 5.0 and sent > done:
                raise RuntimeError("the arm stopped reporting progress")
            if done >= total:
                break
            if not held and sent >= job_from and done >= sent:
                S.run["message"] = "holding %.0f s before drawing" % hold_s
                await asyncio.sleep(hold_s)
                S.run["message"] = ""
                held, stalled = True, time.monotonic()
                continue
            limit = total if held else min(total, job_from)
            burst = 0
            while sent < limit and sent - done < WINDOW and burst < 24:
                k = sent + 1
                rate += float(np.clip(S.speed - rate, -SPEED_SLEW * rate, SPEED_SLEW * rate))
                owed += cfg.sample_ms / rate
                # never ask for more steps than the firmware can generate in the time
                fastest = int(np.ceil(step_change[sent] * 1000.0 / (0.9 * FIRMWARE_TICK_HZ)))
                ms = max(1, fastest, int(round(owed)))
                owed -= ms
                dev.segment(sent, steps[k], ms, rgb[k])
                sent += 1
                burst += 1
            S.run["progress"] = done / total
            if job_from is not None:
                S.run["k"] = max(0, done - job_from)
            await asyncio.sleep(0.01)
        missed = ["%s %.2f°" % (cfg.names[j], abs(w)) for j, w in enumerate(worst)
                  if abs(w) > cfg.slip_warn]
        note = " finished" + (", but missed steps: " + ", ".join(missed) if missed else "")
        S.run = {"state": "idle", "label": "", "progress": 1.0, "k": None, "message": label + note}
        return "done"
    except asyncio.CancelledError:
        dev.abort()
        S.run = {"state": "idle", "label": "", "progress": 0.0, "k": None, "message": label + " stopped"}
        return "stopped"
    except Exception as exc:
        dev.abort()
        S.run = {"state": "idle", "label": "", "progress": 0.0, "k": None,
                 "message": "%s failed: %s" % (label, exc)}
        return "failed"


def check_ready():
    if busy():
        fail(409, "the arm is already moving")
    st = device().status()
    if not st["online"]:
        fail(409, "the arm is not responding")
    if not st["homed"]:
        fail(409, "home the arm first")
    if not st["motors"]:
        fail(409, "the motors are released; hold them first")
    if st["state"] != "idle":
        fail(409, "the arm is busy (%s)" % st["state"])


def start_move(q, rgb, label, job_from=None, hold_s=0.0):
    check_ready()
    S.task = asyncio.create_task(stream(q, rgb, label, job_from, hold_s))


def with_approach(job: planner.Job):
    """The job's samples with a dark move from the arm's current pose put in front.
    Returns (q, rgb, index where the job itself starts)."""
    lead = approach(job.q[0])
    q = np.vstack([lead, job.q[1:]])
    rgb = np.vstack([np.zeros((len(lead), 3), np.uint8), job.rgb[1:]])
    return q, rgb, len(lead) - 1


async def animate(body: dict, first: int, hold_s: float, gap_s: float):
    """Draw every animation frame in turn, one long exposure each. Between frames the arm
    waits gap_s seconds for the camera, or for the Next frame button when gap_s is 0."""
    frames = animation(body)[0]
    idle = {"state": "idle", "label": "", "progress": 0.0, "k": None}
    try:
        for i in range(first, frames):
            S.job, S.frame = build_job(body, i), i
            label = "frame %d of %d" % (i + 1, frames)
            if S.job.stats["strokes"]:
                await trust_encoders()
                q, rgb, job_from = with_approach(S.job)
                result = await stream(q, rgb, label, job_from, hold_s, {"frame": i, "frames": frames})
                if result != "done":
                    return
            if i + 1 < frames:
                wait = gap_s <= 0
                S.run = dict(idle, state="running", label=label, progress=1.0, frame=i, frames=frames,
                             waiting=wait, message="%s done. %s" % (
                                 label, "Press Next frame when the camera is ready." if wait
                                 else "Next frame in %.0f s." % gap_s))
                if wait:
                    S.next_frame = asyncio.Event()
                    await S.next_frame.wait()
                else:
                    await asyncio.sleep(gap_s)
        S.run = dict(idle, message="animation finished: %d frames" % (frames - first))
    except asyncio.CancelledError:
        S.device.abort()
        S.run = dict(idle, message="animation stopped at frame %d" % (S.frame + 1))
    except Exception as exc:
        S.device.abort()
        detail = exc.detail if isinstance(exc, HTTPException) else exc
        S.run = dict(idle, message="animation failed at frame %d: %s" % (S.frame + 1, detail))


def approach(target_q):
    """Samples for a dark move from where the arm is now to target_q (first row = now)."""
    now = current_q()
    lo, hi = cfg.q_min - 0.5, cfg.q_max + 0.5
    if np.any((now < lo) | (now > hi)):
        fail(409, "the arm reports a pose outside its joint limits; home it again")
    return np.vstack([now[None, :], planner.travel(cfg, now, target_q)])


@app.get("/api/ports")
def api_ports():
    return [{"port": "sim", "label": "Simulator (no hardware)"}] + list_serial_ports()


@app.post("/api/connect")
async def api_connect(body: dict):
    await api_disconnect()
    port = body.get("port", "sim")
    try:
        S.device = SimDevice(cfg) if port == "sim" else SerialDevice(cfg, port)
    except Exception as exc:
        fail(502, "could not open %s: %s" % (port, exc))
    return {"device": S.device.name}


@app.post("/api/disconnect")
async def api_disconnect():
    if busy():
        S.task.cancel()
        await asyncio.wait([S.task])
    if S.device is not None:
        S.device.close()
        S.device = None
    return {}


@app.post("/api/home")
async def api_home():
    if busy():
        fail(409, "the arm is moving")
    device().home()
    return {}


@app.post("/api/save_marks")
async def api_save_marks():
    """Every joint is sitting exactly on its home mark: remember the encoder angles there."""
    if busy():
        fail(409, "the arm is moving")
    device().save_marks()
    return {}


@app.post("/api/home_marks")
async def api_home_marks():
    """Every joint is near its home mark: let the encoders work out the exact position."""
    if busy():
        fail(409, "the arm is moving")
    device().home_marks()
    return {}


@app.post("/api/resync")
async def api_resync():
    if busy():
        fail(409, "the arm is moving")
    device().resync()
    return {}


@app.post("/api/motors")
async def api_motors(body: dict):
    if busy():
        fail(409, "the arm is moving")
    device().motors(bool(body.get("on", True)))
    return {}


@app.post("/api/sim_slip")
async def api_sim_slip(body: dict):
    """Simulator only: fake missed steps, to see how the viewer reports them."""
    if not isinstance(S.device, SimDevice):
        fail(409, "only the simulator can fake missed steps")
    S.device.fake_slip(int(body["joint"]), float(body["deg"]) * cfg.steps_per_deg)
    return {}


@app.post("/api/jog")
async def api_jog(body: dict):
    joint, delta = int(body["joint"]), float(body["delta"])
    st = device().status()
    if not st["homed"]:
        # Before homing nothing is known about the pose, so no limits can be applied:
        # a plain slow relative move, for lining a joint up with its mark.
        if busy() or st["state"] != "idle" or not st["motors"]:
            fail(409, "the arm is busy or its motors are released")
        delta = float(np.clip(delta, -10, 10))
        device().jog(joint, int(round(cfg.direction[joint] * delta * cfg.steps_per_deg)))
        return {}
    await trust_encoders()
    target = current_q()
    target[joint] = np.clip(target[joint] + delta, cfg.q_min[joint], cfg.q_max[joint])
    q = approach(target)
    start_move(q, np.zeros((len(q), 3), np.uint8), "jog")
    return {}


@app.post("/api/park")
async def api_park():
    await trust_encoders()
    q = approach(cfg.q_home)
    start_move(q, np.zeros((len(q), 3), np.uint8), "move to home pose")
    return {}


@app.post("/api/led")
async def api_led(body: dict):
    device().led(parse_hex(body.get("color", "#000000")))
    return {}


@app.post("/api/run")
async def api_run(body: dict):
    job = S.job
    if job is None or len(job.q) < 2:
        fail(409, "nothing to draw: load a file and make sure some of it is reachable")
    await trust_encoders()
    q, rgb, job_from = with_approach(job)
    start_move(q, rgb, "drawing", job_from=job_from,
               hold_s=float(np.clip(body.get("hold_s", 0), 0, 120)))
    return {}


@app.post("/api/animate")
async def api_animate(body: dict):
    """Draw the animation from the given frame onward. body is the viewer's settings plus
    frame (where to start), hold_s and gap_s."""
    frames = animation(body)[0]
    if frames < 2:
        fail(409, "set more than one frame to animate")
    load_source(body.get("source", {}))
    check_ready()
    first = int(np.clip(int(body.get("frame", 0)), 0, frames - 1))
    S.task = asyncio.create_task(animate(
        body, first, float(np.clip(body.get("hold_s", 0), 0, 120)),
        float(np.clip(body.get("gap_s", 0), 0, 600))))
    return {}


@app.post("/api/next_frame")
async def api_next_frame():
    if S.next_frame is not None:
        S.next_frame.set()
    return {}


@app.post("/api/speed")
async def api_speed(body: dict):
    """Live speed control, as a fraction of the planned speed. Takes effect within about a
    second during a move (the segments already queued on the arm play out first)."""
    S.speed = float(np.clip(float(body["factor"]), SPEED_MIN, SPEED_MAX))
    return {"speed": S.speed}


@app.post("/api/stop")
async def api_stop():
    if busy():
        S.task.cancel()
    elif S.device is not None:
        S.device.abort()
    return {}


@app.get("/api/config")
def api_config():
    return cfg.public()


@app.websocket("/ws")
async def ws(sock: WebSocket):
    """Push the arm's reported pose and run progress to the viewer 25 times a second."""
    await sock.accept()
    try:
        while True:
            msg = {"connected": S.device is not None, "run": S.run, "speed": S.speed}
            dev = S.device
            if dev is not None:
                st = dev.status()
                q = cfg.steps_to_deg(st["steps"])
                log = list(dev.log)
                msg.update({
                    "device": dev.name, "q": np.round(q, 2).tolist(),
                    "tip": np.round(fk(cfg, q), 1).tolist(), "rgb": st["rgb"],
                    "homed": st["homed"], "state": st["state"], "online": st["online"],
                    "queued": st["queued"], "error": dev.error,
                    "slip": [None if d is None else round(d, 3) for d in slip_deg(st)],
                    "slipped": slipped(st), "motors": st["motors"],
                    "log": log[-8:],
                })
            await sock.send_json(msg)
            await asyncio.sleep(0.04)
    except (WebSocketDisconnect, RuntimeError):
        pass


class FreshFiles(StaticFiles):
    """Static files that the browser must re-check every time, so an updated viewer or
    example is never served stale from its cache."""

    async def get_response(self, path, scope):
        response = await super().get_response(path, scope)
        response.headers["Cache-Control"] = "no-cache"
        return response


app.mount("/examples", FreshFiles(directory=str(ROOT / "examples")), name="examples")
app.mount("/", FreshFiles(directory=str(ROOT / "web"), html=True), name="web")
