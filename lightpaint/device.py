"""The arm as the PC sees it: a queue of timed step segments plus a status report.

SerialDevice talks to the ESP32 firmware; SimDevice behaves the same way in software so
the whole program can be used with no hardware attached. The serial protocol is described at the top of firmware/src/main.cpp.
"""
from __future__ import annotations

import threading
import time
from collections import deque

import numpy as np
import serial
from serial.tools import list_ports

QUEUE_SIZE = 64       # must match QUEUE_SIZE in firmware/src/config.h
STALE_AFTER_S = 1.0   # no status line for this long means the link is down


def list_serial_ports():
    return [{"port": p.device, "label": "%s (%s)" % (p.device, p.description)}
            for p in list_ports.comports()]


class SimDevice:
    name = "Simulator"

    def __init__(self, cfg):
        self.cfg = cfg
        self._pos = np.zeros(3)
        self._queue = deque()     # (t_start, t_end, from_steps, to_steps, rgb, counts_as_done)
        self._done = 0
        self._homed = True
        self._idle_rgb = (0, 0, 0)
        self._homing_until = 0.0
        self._slip = np.zeros(3)      # pretend missed steps: encoder position minus step count
        self._motors = True
        self.error = None
        self.log = deque(maxlen=200)
        self.log.append("simulator ready (already homed)")

    def _advance(self):
        now = time.monotonic()
        while self._queue and self._queue[0][1] <= now:
            seg = self._queue.popleft()
            self._pos = seg[3]
            self._done += seg[5]
        return now

    def status(self) -> dict:
        now = self._advance()
        pos, rgb = self._pos, self._idle_rgb
        if self._queue and self._queue[0][0] <= now:
            t0, t1, a, b, rgb, _ = self._queue[0]
            pos = a + (b - a) * (now - t0) / (t1 - t0)
        homing = now < self._homing_until
        return {
            "steps": [float(v) for v in pos],
            "done": self._done,
            "queued": len(self._queue),
            "homed": self._homed,
            "state": "home" if homing else ("run" if self._queue else "idle"),
            "rgb": [int(c) for c in rgb],
            "online": True,
            "enc": [True, True, True],
            "slip": [float(v) for v in self._slip],
            "motors": self._motors,
        }

    def _push(self, target, seconds, rgb, counts=1):
        now = self._advance()
        start = self._queue[-1][1] if self._queue else now
        origin = self._queue[-1][3] if self._queue else self._pos
        self._queue.append((start, start + seconds, origin, np.asarray(target, float), rgb, counts))

    def new_job(self):
        self._advance()
        self._done = 0
        self.error = None

    def segment(self, seq, steps, ms, rgb):
        self._push(steps, ms / 1000.0, tuple(rgb))

    def abort(self):
        st = self.status()
        self._pos = np.array(st["steps"])
        self._queue.clear()
        self._idle_rgb = (0, 0, 0)
        self._homing_until = 0.0

    def home(self):
        self.abort()
        self._push(np.zeros(3), 2.0, (0, 0, 0), counts=0)
        self._homing_until = time.monotonic() + 2.0
        self._homed = True

    def set_position(self, steps):
        self.abort()
        self._pos = np.asarray(steps, float)
        self._slip = np.zeros(3)
        self._homed = True

    def save_marks(self):
        self.set_position(np.zeros(3))
        self.log.append("marks saved: this pose is home")

    def home_marks(self):
        self.set_position(np.zeros(3))
        self.log.append("homed at marks")

    def resync(self):
        self.abort()
        self._pos = self._pos + self._slip
        self._slip = np.zeros(3)
        self.log.append("resynced to encoders")

    def jog(self, joint, steps):
        target = np.array(self.status()["steps"])
        target[joint] += steps
        self._push(target, abs(steps) / 1600.0, (0, 0, 0), counts=0)
        self._homing_until = time.monotonic() + abs(steps) / 1600.0

    def motors(self, on):
        self._motors = bool(on)

    def fake_slip(self, joint, steps):
        """Simulator only: act as if this joint's motor just missed some steps."""
        self._slip[joint] += steps

    def led(self, rgb):
        self._idle_rgb = tuple(rgb)

    def close(self):
        pass


class SerialDevice:
    def __init__(self, cfg, port: str):
        self.cfg = cfg
        self.name = port
        self.error = None
        self.log = deque(maxlen=200)
        self._status = {"steps": [0.0, 0.0, 0.0], "done": 0, "queued": 0, "homed": False,
                        "state": "idle", "rgb": [0, 0, 0], "enc": [False] * 3,
                        "slip": [0.0] * 3, "motors": True}
        self._seen = 0.0
        self._lock = threading.Lock()
        self._open = True
        self._ser = serial.Serial(port, cfg.baud, timeout=0.2)
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self):
        buf = b""
        while self._open:
            try:
                chunk = self._ser.read(256)
            except (serial.SerialException, OSError) as exc:
                self.error = "serial link lost: %s" % exc
                self.log.append(self.error)
                return
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self._handle(line.decode("ascii", "replace").strip())

    def _handle(self, line: str):
        if not line:
            return
        if line.startswith("st "):
            f = line.split()
            if len(f) != 16:
                return
            try:
                v = [int(x) for x in f[1:6]] + [int(x) for x in f[8:16]]
            except ValueError:
                return
            self._status = {
                "steps": [float(x) for x in v[0:3]], "done": v[3], "queued": v[4],
                "homed": f[6] == "7",
                "state": {"I": "idle", "R": "run", "H": "home"}.get(f[7], "idle"),
                "rgb": v[5:8],
                "enc": [bool(v[8] >> j & 1) for j in range(3)],
                "slip": [float(x) for x in v[9:12]],
                "motors": v[12] == 1,
            }
            self._seen = time.monotonic()
            return
        self.log.append(line)
        if line.startswith("err "):
            self.error = line[4:]

    def _send(self, text: str):
        with self._lock:
            self._ser.write((text + "\n").encode("ascii"))

    def status(self) -> dict:
        st = dict(self._status)
        st["online"] = time.monotonic() - self._seen < STALE_AFTER_S
        return st

    def new_job(self):
        self.error = None
        self._status["done"] = 0
        self._send("N")

    def segment(self, seq, steps, ms, rgb):
        self._send("S %d %d %d %d %d %d %d %d" % (seq, steps[0], steps[1], steps[2], ms, *rgb))

    def abort(self):
        self._send("X")

    def home(self):
        self.error = None
        self._send("H")

    def set_position(self, steps):
        self.error = None
        self._send("Z %d %d %d" % tuple(int(s) for s in steps))

    def save_marks(self):
        self.error = None
        self._send("M")

    def home_marks(self):
        self.error = None
        self._send("K")

    def resync(self):
        self.error = None
        self._send("C")

    def jog(self, joint, steps):
        self.error = None
        self._send("J %d %d" % (joint, steps))

    def motors(self, on):
        self._send("E %d" % (1 if on else 0))

    def led(self, rgb):
        self._send("L %d %d %d" % tuple(rgb))

    def close(self):
        self._open = False
        try:
            self._ser.close()
        except Exception:
            pass
