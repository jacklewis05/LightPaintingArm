"""Forward and inverse kinematics for the base / shoulder / elbow arm (vectorised)."""
from __future__ import annotations

import numpy as np

from .config import ArmConfig


def fk(cfg: ArmConfig, q_deg) -> np.ndarray:
    """Joint angles (..., 3) in degrees -> LED tip position (..., 3) in mm."""
    q = np.radians(np.asarray(q_deg, float))
    q1, q2, q3 = q[..., 0], q[..., 1], q[..., 2]
    r = cfg.a1 + cfg.l1 * np.cos(q2) + cfg.l2 * np.cos(q2 + q3)
    z = cfg.h + cfg.l1 * np.sin(q2) + cfg.l2 * np.sin(q2 + q3)
    c, s = np.cos(q1), np.sin(q1)
    return np.stack([r * c - cfg.d * s, r * s + cfg.d * c, z], axis=-1)


def ik(cfg: ArmConfig, pts):
    """Tip positions (N, 3) -> (joint angles (N, 3) in degrees, reachable mask (N,)).

    A point is reachable when the geometry closes, every joint is inside its limits
    and the tip stays above min_tip_height. Angles of unreachable points are not meaningful.
    """
    p = np.atleast_2d(np.asarray(pts, float))
    x, y, z = p[:, 0], p[:, 1], p[:, 2]

    rxy2 = x * x + y * y
    ok = rxy2 > cfg.d * cfg.d + 1e-9
    r = np.sqrt(np.maximum(rxy2 - cfg.d * cfg.d, 0.0))
    q1 = np.arctan2(y, x) - np.arctan2(cfg.d, r)
    q1 = (q1 + np.pi) % (2 * np.pi) - np.pi

    rr = r - cfg.a1
    zz = z - cfg.h
    c3 = (rr * rr + zz * zz - cfg.l1**2 - cfg.l2**2) / (2 * cfg.l1 * cfg.l2)
    ok &= np.abs(c3) <= 1.0
    q3 = cfg.elbow_sign * np.arccos(np.clip(c3, -1.0, 1.0))
    q2 = np.arctan2(zz, rr) - np.arctan2(cfg.l2 * np.sin(q3), cfg.l1 + cfg.l2 * np.cos(q3))

    q = np.degrees(np.stack([q1, q2, q3], axis=1))
    ok &= np.all((q >= cfg.q_min - 1e-9) & (q <= cfg.q_max + 1e-9), axis=1)
    ok &= z >= cfg.min_z
    return q, ok
