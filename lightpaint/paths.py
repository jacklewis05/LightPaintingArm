"""Strokes (coloured 3D polylines) and the operations shared by every input type."""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

PALETTE = [(255, 40, 20), (40, 255, 60), (60, 110, 255), (255, 255, 255),
           (255, 200, 0), (0, 230, 255), (255, 0, 200)]


@dataclass
class Stroke:
    pts: np.ndarray                       # (N, 3); a closed stroke repeats its first point at the end
    rgb: Tuple[int, int, int] = (255, 255, 255)
    closed: bool = False


def length(pts: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())


def normalize(strokes: List[Stroke]):
    """Centre the drawing on the origin and scale its largest dimension to 1.

    Returns (strokes, original bounding-box size)."""
    allp = np.vstack([s.pts for s in strokes])
    lo, hi = allp.min(axis=0), allp.max(axis=0)
    size = hi - lo
    scale = float(size.max()) or 1.0
    centre = (lo + hi) / 2
    return [Stroke((s.pts - centre) / scale, s.rgb, s.closed) for s in strokes], size


def rotation(rx: float, ry: float, rz: float) -> np.ndarray:
    """Rotation matrix for rotating about X, then Y, then Z (degrees, world axes)."""
    a, b, c = np.radians([rx, ry, rz])
    mx = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
    my = np.array([[np.cos(b), 0, np.sin(b)], [0, 1, 0], [-np.sin(b), 0, np.cos(b)]])
    mz = np.array([[np.cos(c), -np.sin(c), 0], [np.sin(c), np.cos(c), 0], [0, 0, 1]])
    return mz @ my @ mx


def spin(axis: str, deg: float) -> np.ndarray:
    """Rotation by deg about one world axis ('x', 'y' or 'z')."""
    return rotation(*[deg if a == axis else 0.0 for a in "xyz"])


def place(strokes: List[Stroke], pl: dict) -> List[Stroke]:
    """Normalised strokes -> world millimetres using a placement dict.

    The optional spin_axis / spin_deg turn the drawing about its own centre after the
    rx, ry, rz rotation; animation frames differ only in spin_deg."""
    rot = rotation(pl.get("rx", 0), pl.get("ry", 0), pl.get("rz", 0))
    rot = spin(pl.get("spin_axis", "z"), pl.get("spin_deg", 0.0)) @ rot
    centre = np.array([pl["x"], pl["y"], pl["z"]], float)
    return [Stroke((s.pts * pl["size"]) @ rot.T + centre, s.rgb, s.closed) for s in strokes]


def colorize(strokes: List[Stroke], mode: str, color) -> List[Stroke]:
    """mode: 'single' (one colour), 'palette' (cycle per stroke) or 'source' (keep loaded colours)."""
    if mode == "single":
        rgb = tuple(int(c) for c in color)
        return [Stroke(s.pts, rgb, s.closed) for s in strokes]
    if mode == "palette":
        return [Stroke(s.pts, PALETTE[i % len(PALETTE)], s.closed) for i, s in enumerate(strokes)]
    return strokes


def parse_hex(color: str) -> Tuple[int, int, int]:
    c = color.lstrip("#")
    return int(c[0:2], 16), int(c[2:4], 16), int(c[4:6], 16)
