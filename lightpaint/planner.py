"""Turn world-space strokes into a timed joint trajectory the firmware can replay.

The output is a list of samples spaced one firmware segment (sample_ms) apart. Each sample
holds the joint angles to reach by the end of that interval and the LED colour to show
during it. All speed and acceleration limiting happens here, so the firmware only has to
interpolate linearly between consecutive samples.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from .config import ArmConfig
from .kinematics import fk, ik
from .paths import Stroke

MAX_JOINT_JUMP_DEG = 15.0   # a bigger jump between neighbouring points means the base wrapped around


@dataclass
class Run:
    """A reachable piece of a stroke, densely sampled."""
    pts: np.ndarray
    q: np.ndarray
    stroke: int
    closed: bool


@dataclass
class Job:
    dt: float
    q: np.ndarray            # (K, 3) degrees; row 0 is the start pose
    tip: np.ndarray          # (K, 3) mm
    rgb: np.ndarray          # (K, 3) LED colour during the interval that ends at each sample
    path: np.ndarray         # (K,) stroke index for that interval, -1 while travelling dark
    stroke_rgb: List[tuple]
    unreachable: List[np.ndarray] = field(default_factory=list)
    stats: dict = field(default_factory=dict)


def densify(pts: np.ndarray, step: float) -> np.ndarray:
    """Split every segment into pieces no longer than step (at least two per segment)."""
    out = [pts[:1]]
    for a, b in zip(pts[:-1], pts[1:]):
        dist = np.linalg.norm(b - a)
        if dist < 1e-9:
            continue
        n = max(2, int(np.ceil(dist / step)))
        t = np.arange(1, n + 1)[:, None] / n
        out.append(a + (b - a) * t)
    return np.vstack(out)


def split_reachable(cfg: ArmConfig, strokes: List[Stroke], step: float):
    """IK every stroke and cut it into reachable runs. Returns (runs, unreachable polylines)."""
    runs, bad = [], []
    for si, s in enumerate(strokes):
        if len(s.pts) < 2:
            continue
        pts = densify(s.pts, step)
        if len(pts) < 2:
            continue
        q, ok = ik(cfg, pts)
        jump = np.abs(np.diff(q, axis=0)).max(axis=1) > MAX_JOINT_JUMP_DEG
        # link[i] is True when the arm can draw from point i to point i + 1
        link = ok[:-1] & ok[1:] & ~jump
        if link.all():
            runs.append(Run(pts, q, si, s.closed))
            continue
        edges = np.flatnonzero(np.diff(np.concatenate([[False], link, [False]]).astype(int)))
        for a, b in zip(edges[::2], edges[1::2]):
            runs.append(Run(pts[a:b + 1], q[a:b + 1], si, False))
        gaps = np.flatnonzero(np.diff(np.concatenate([[False], ~link, [False]]).astype(int)))
        for a, b in zip(gaps[::2], gaps[1::2]):
            bad.append(pts[a:b + 1])
    return runs, bad


def order_runs(runs: List[Run], start: np.ndarray) -> List[Run]:
    """Greedy nearest-neighbour ordering; open runs may be reversed, loops may start anywhere."""
    if not runs:
        return []
    firsts = np.array([r.pts[0] for r in runs])
    lasts = np.array([r.pts[-1] for r in runs])
    todo = np.ones(len(runs), bool)
    cur, out = np.asarray(start, float), []
    for _ in range(len(runs)):
        d_first = np.where(todo, np.linalg.norm(firsts - cur, axis=1), np.inf)
        d_last = np.where(todo, np.linalg.norm(lasts - cur, axis=1), np.inf)
        i = int(np.argmin(np.minimum(d_first, d_last)))
        r = runs[i]
        todo[i] = False
        if r.closed:
            k = int(np.argmin(np.linalg.norm(r.pts[:-1] - cur, axis=1)))
            if k:
                r = Run(np.vstack([r.pts[k:-1], r.pts[:k + 1]]),
                        np.vstack([r.q[k:-1], r.q[:k + 1]]), r.stroke, True)
        elif d_last[i] < d_first[i]:
            r = Run(r.pts[::-1], r.q[::-1], r.stroke, False)
        out.append(r)
        cur = r.pts[-1]
    return out


def travel(cfg: ArmConfig, qa, qb, speed: float = 1.0) -> np.ndarray:
    """Dark joint-space move qa -> qb with a trapezoidal speed profile.

    Returns the samples after qa, spaced cfg.dt apart (empty when qa == qb)."""
    qa, qb = np.asarray(qa, float), np.asarray(qb, float)
    delta = qb - qa
    mag = np.abs(delta)
    if mag.max() < 1e-6:
        return np.zeros((0, 3))
    moving = mag > 1e-9
    # Work with progress u in 0..1: limits on du/dt and d2u/dt2 that keep every joint legal.
    v = float(np.min(cfg.v_max[moving] / mag[moving])) * speed
    a = float(np.min(cfg.a_max[moving] / mag[moving]))
    t_ramp = v / a
    if a * t_ramp**2 >= 1.0:          # never reaches cruise speed
        t_ramp = np.sqrt(1.0 / a)
        v = a * t_ramp
        total = 2 * t_ramp
    else:
        total = 1.0 / v + t_ramp
    n = max(1, int(np.ceil(total / cfg.dt)))
    t = np.arange(1, n + 1) * (total / n)
    u = np.where(
        t < t_ramp, 0.5 * a * t**2,
        np.where(t <= total - t_ramp, v * (t - t_ramp) + 0.5 * a * t_ramp**2,
                 1.0 - 0.5 * a * (total - t) ** 2))
    u[-1] = 1.0
    return qa + np.clip(u, 0.0, 1.0)[:, None] * delta


def draw(cfg: ArmConfig, pts: np.ndarray, q: np.ndarray, speed: float):
    """Lit move along a dense polyline: full speed on straights, slowing for corners and for
    joint speed limits, starting and ending at rest.

    Returns (joint samples after the first point, tip speed in mm/s over each interval)."""
    if len(pts) == 2:   # give the speed profile an interior point to accelerate towards
        pts = np.vstack([pts[0], pts.mean(axis=0), pts[1]])
        q = np.vstack([q[0], q.mean(axis=0), q[1]])
    seg = np.diff(pts, axis=0)
    ds = np.linalg.norm(seg, axis=1)
    n = len(ds)
    dq = np.abs(np.diff(q, axis=0))
    with np.errstate(divide="ignore"):
        v_seg = np.minimum(speed, np.min(cfg.v_max * ds[:, None] / dq, axis=1))
        a_seg = np.minimum(cfg.tip_accel, np.min(cfg.a_max * ds[:, None] / dq, axis=1))

    # Speed allowed through each interior point. Where the path bends, each joint's speed
    # changes by (tip speed) x (change in degrees-per-mm). A motor can take a small jump
    # instantly (max_jump_dps); gentler bends spread over the neighbouring segments must
    # also stay inside the joint's acceleration limit.
    slope = np.diff(q, axis=0) / ds[:, None]              # degrees of joint per mm of path
    bend = np.abs(np.diff(slope, axis=0))
    span = 0.5 * (ds[:-1] + ds[1:])[:, None]
    with np.errstate(divide="ignore"):
        v_jump = np.min(cfg.v_jump / bend, axis=1)
        v_curve = np.sqrt(np.min(cfg.a_max * span / bend, axis=1))
    v_corner = np.minimum(v_jump, v_curve)

    v = np.zeros(n + 1)
    v[1:n] = np.minimum(v_corner, np.minimum(v_seg[:-1], v_seg[1:]))
    for i in range(n):                      # can't arrive faster than acceleration allows
        v[i + 1] = min(v[i + 1], np.sqrt(v[i] ** 2 + 2 * a_seg[i] * ds[i]))
    for i in range(n - 1, -1, -1):          # can't leave faster than braking allows
        v[i] = min(v[i], np.sqrt(v[i + 1] ** 2 + 2 * a_seg[i] * ds[i]))

    seg_t = 2 * ds / np.maximum(v[:-1] + v[1:], 1e-6)
    t_at = np.concatenate([[0.0], np.cumsum(seg_t)])
    total = t_at[-1]
    k = max(1, int(np.ceil(total / cfg.dt)))
    t = np.arange(1, k + 1) * (total / k)
    i = np.clip(np.searchsorted(t_at, t, side="right") - 1, 0, n - 1)
    tau = t - t_at[i]
    accel = (v[i + 1] ** 2 - v[i] ** 2) / (2 * ds[i])
    frac = np.clip((v[i] * tau + 0.5 * accel * tau**2) / ds[i], 0.0, 1.0)[:, None]
    frac[-1] = 1.0
    i[-1] = n - 1
    q_out = q[i] + (q[i + 1] - q[i]) * frac
    p_out = pts[i] + (pts[i + 1] - pts[i]) * frac
    step = np.linalg.norm(np.diff(np.vstack([pts[:1], p_out]), axis=0), axis=1)
    return q_out, step / (total / k)


def plan(cfg: ArmConfig, strokes: List[Stroke], draw_speed: float = 40.0,
         brightness: float = 1.0, even_exposure: bool = True, return_home: bool = True,
         start_q: Optional[np.ndarray] = None) -> Job:
    """Build the whole job: travel dark to each stroke, draw it lit, and optionally park."""
    start_q = cfg.q_home if start_q is None else np.asarray(start_q, float)
    runs, unreachable = split_reachable(cfg, strokes, cfg.resample)
    runs = order_runs(runs, fk(cfg, start_q))

    q_parts, rgb_parts, path_parts = [start_q[None, :]], [np.zeros((1, 3))], [np.array([-1])]
    cur = start_q
    draw_len = draw_time = travel_time = 0.0

    def add_travel(target):
        nonlocal cur, travel_time
        samples = travel(cfg, cur, target)
        if len(samples):
            q_parts.append(samples)
            rgb_parts.append(np.zeros((len(samples), 3)))
            path_parts.append(np.full(len(samples), -1))
            travel_time += len(samples) * cfg.dt
        cur = np.asarray(target, float)

    for r in runs:
        add_travel(r.q[0])
        samples, tip_speed = draw(cfg, r.pts, r.q, draw_speed)
        # Light reaching the camera per mm of line is brightness / speed, so dimming in
        # proportion to speed keeps corners and stroke ends from burning in as bright dots.
        level = np.clip(tip_speed / draw_speed, 0.0, 1.0) if even_exposure else np.ones(len(samples))
        colour = np.array(strokes[r.stroke].rgb, float)
        q_parts.append(samples)
        rgb_parts.append(np.outer(level * brightness, colour))
        path_parts.append(np.full(len(samples), r.stroke))
        draw_len += float(np.linalg.norm(np.diff(r.pts, axis=0), axis=1).sum())
        draw_time += len(samples) * cfg.dt
        cur = r.q[-1]
    if return_home and runs:
        add_travel(cfg.q_home)

    q = np.vstack(q_parts)
    tip = fk(cfg, q)
    lost = float(sum(np.linalg.norm(np.diff(u, axis=0), axis=1).sum() for u in unreachable))
    stats = {
        "strokes": len(runs),
        "samples": len(q),
        "draw_mm": round(draw_len, 1),
        "unreachable_mm": round(lost, 1),
        "draw_s": round(draw_time, 2),
        "travel_s": round(travel_time, 2),
        "total_s": round((len(q) - 1) * cfg.dt, 2),
        "below_table": bool((tip[:, 2] < cfg.min_z - 1e-6).any()),
    }
    return Job(cfg.dt, q, tip, np.clip(np.rint(np.vstack(rgb_parts)), 0, 255).astype(np.uint8),
               np.concatenate(path_parts), [tuple(s.rgb) for s in strokes], unreachable, stats)


def autofit(cfg: ArmConfig, strokes: List[Stroke], rot: dict, max_size: float = 400.0,
            spins=(("z", 0.0),)) -> dict:
    """Find the placement (centre in the X-Z plane, plus size) that draws the normalised
    strokes as large as possible with every point reachable.

    spins lists the (axis, degrees) turn of every animation frame, so that one placement
    fits them all."""
    from .paths import rotation, spin

    pts = np.vstack([densify(s.pts, 0.03) for s in strokes if len(s.pts) > 1])
    if len(pts) > 6000:     # a sample is enough to find the fit; planning checks every point
        pts = pts[np.random.default_rng(0).choice(len(pts), 6000, replace=False)]
    pts = pts @ rotation(rot.get("rx", 0), rot.get("ry", 0), rot.get("rz", 0)).T
    spins = list(spins)[:: max(1, len(spins) // 12)]        # a dozen frames is enough to bound them all
    if len(pts) * len(spins) > 12000:
        pts = pts[:: int(np.ceil(len(pts) * len(spins) / 12000))]
    pts = np.vstack([pts @ spin(axis, deg).T for axis, deg in spins])
    reach = cfg.l1 + cfg.l2

    def fits(x, z, size):
        return bool(ik(cfg, pts * size + np.array([x, 0.0, z]))[1].all())

    best = (0.0, cfg.a1 + 0.6 * reach, cfg.h + 0.2 * reach)
    for x in cfg.a1 + reach * np.linspace(0.35, 0.85, 6):
        for z in cfg.h + reach * np.linspace(-0.1, 0.6, 8):
            lo, hi = 0.0, max_size
            if not fits(x, z, 10.0):
                continue
            lo = 10.0
            for _ in range(8):
                mid = (lo + hi) / 2
                lo, hi = (mid, hi) if fits(x, z, mid) else (lo, mid)
            if lo > best[0]:
                best = (lo, x, z)
    size = best[0] * 0.92 if best[0] > 0 else 100.0     # leave a little margin from the limits
    return dict(rot, x=round(best[1], 1), y=0.0, z=round(best[2], 1), size=round(size, 1))
