"""Arm description loaded from arm_config.json.

Conventions (all on the PC side; the firmware only knows motor steps):
  World frame: origin on the table under the base axis, X forward, Y left, Z up.
  q1 base     : yaw about Z, 0 = arm pointing along +X, positive = counter-clockwise from above.
  q2 shoulder : elevation of the upper arm above horizontal (90 = straight up).
  q3 elbow    : forearm angle relative to the upper arm (0 = straight, negative = folded down).
  Motor steps : 0 at the hall-sensor trigger, i.e. at each joint's home_deg.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_PATH = ROOT / "arm_config.json"

# Must match TICK_HZ in firmware/src/config.h: one step per tick is the hard ceiling.
FIRMWARE_TICK_HZ = 40000


class ArmConfig:
    def __init__(self, raw: dict):
        self.raw = raw
        g = raw["geometry_mm"]
        self.h = float(g["shoulder_height"])
        self.a1 = float(g["shoulder_forward"])
        self.d = float(g["lateral_offset"])
        self.l1 = float(g["upper_arm"])
        self.l2 = float(g["forearm"])
        self.min_z = float(g["min_tip_height"])
        self.elbow_sign = -1.0 if raw.get("elbow", "up") == "up" else 1.0

        j = raw["joints"]
        if len(j) != 3:
            raise ValueError("arm_config.json must describe exactly 3 joints")
        self.names = [x["name"] for x in j]
        self.q_min = np.array([x["min_deg"] for x in j], float)
        self.q_max = np.array([x["max_deg"] for x in j], float)
        self.q_home = np.array([x["home_deg"] for x in j], float)
        self.direction = np.array([x["direction"] for x in j], float)
        self.v_max = np.array([x["max_speed_dps"] for x in j], float)
        self.a_max = np.array([x["max_accel_dps2"] for x in j], float)
        self.v_jump = np.array([x["max_jump_dps"] for x in j], float)

        dr = raw["drive"]
        self.steps_per_deg = (
            dr["motor_steps_per_rev"] * dr["microsteps"] * dr["gear_ratio"] / 360.0
        )

        m = raw["motion"]
        self.sample_ms = int(m["sample_ms"])
        self.dt = self.sample_ms / 1000.0
        self.tip_accel = float(m["tip_accel_mm_s2"])
        self.resample = float(m["resample_mm"])
        self.baud = int(raw["serial"]["baud"])
        # Joint error (encoder vs. step count) above this counts as missed steps. A stalled
        # stepper drops back by whole multiples of 4 full steps, about 0.25 deg at the joint.
        self.slip_warn = float(raw.get("encoders", {}).get("slip_warn_deg", 0.15))

        self.warnings = []
        if np.any((self.q_home < self.q_min) | (self.q_home > self.q_max)):
            raise ValueError("every joint's home_deg must lie inside its min/max limits")
        rate = self.v_max * self.steps_per_deg
        if np.any(rate > 0.8 * FIRMWARE_TICK_HZ):
            raise ValueError(
                "max_speed_dps needs %.0f steps/s, above what the firmware can generate (%d)"
                % (rate.max(), int(0.8 * FIRMWARE_TICK_HZ))
            )

    @classmethod
    def load(cls, path: Path = DEFAULT_PATH) -> "ArmConfig":
        return cls(json.loads(Path(path).read_text()))

    def deg_to_steps(self, q) -> np.ndarray:
        q = np.asarray(q, float)
        return np.rint(self.direction * (q - self.q_home) * self.steps_per_deg).astype(np.int64)

    def steps_to_deg(self, steps) -> np.ndarray:
        steps = np.asarray(steps, float)
        return self.q_home + self.direction * steps / self.steps_per_deg

    def public(self) -> dict:
        """What the browser needs to draw the arm."""
        return {
            "geometry": {"h": self.h, "a1": self.a1, "d": self.d, "l1": self.l1, "l2": self.l2},
            "joints": [
                {"name": n, "min": lo, "max": hi, "home": hm}
                for n, lo, hi, hm in zip(self.names, self.q_min, self.q_max, self.q_home)
            ],
            "dt": self.dt,
        }
