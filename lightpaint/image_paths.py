"""Bitmap image -> strokes on a flat plane.

Three tracing modes:
  edges      Canny edge detection, good for photos.
  outline    threshold, then the outline of every dark shape; good for logos and silhouettes.
  centerline threshold, thin every dark line to one pixel, trace it; good for line drawings.

Strokes come out in the Y-Z plane (image right = +Y, image up = +Z) so a camera standing in
front of the arm (on +X, looking back at it) sees the picture the right way round.
"""
from __future__ import annotations

from typing import List

import cv2
import numpy as np

from .paths import Stroke

IMAGE_TYPES = {"png", "jpg", "jpeg", "bmp", "webp", "gif", "tif", "tiff"}
MAX_DIM = 700
MAX_STROKES = 1500

# Neighbour offsets, 4-connected first so traces prefer straight moves over diagonal ones.
_NEIGHBOURS = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]


def _decode(data: bytes) -> np.ndarray:
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_UNCHANGED)
    if img is None:
        raise ValueError("could not read this image file")
    if img.dtype != np.uint8:
        img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    elif img.shape[2] == 4:  # flatten transparency onto white
        alpha = img[:, :, 3:4].astype(np.float32) / 255.0
        img = (img[:, :, :3] * alpha + 255.0 * (1.0 - alpha)).astype(np.uint8)
    scale = MAX_DIM / max(img.shape[:2])
    if scale < 1.0:
        img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    return img


def thin(mask: np.ndarray) -> np.ndarray:
    """Zhang-Suen thinning: reduce a boolean mask to one-pixel-wide centre lines."""
    img = mask.astype(np.uint8)
    for _ in range(200):
        changed = False
        for step in (0, 1):
            p = np.pad(img, 1)
            p2, p3, p4 = p[:-2, 1:-1], p[:-2, 2:], p[1:-1, 2:]
            p5, p6, p7 = p[2:, 2:], p[2:, 1:-1], p[2:, :-2]
            p8, p9 = p[1:-1, :-2], p[:-2, :-2]
            ring = [p2, p3, p4, p5, p6, p7, p8, p9, p2]
            count = sum(ring[:8])
            transitions = sum((ring[i] == 0) & (ring[i + 1] == 1) for i in range(8))
            if step == 0:
                cond = (p2 * p4 * p6 == 0) & (p4 * p6 * p8 == 0)
            else:
                cond = (p2 * p4 * p8 == 0) & (p2 * p6 * p8 == 0)
            remove = (img == 1) & (count >= 2) & (count <= 6) & (transitions == 1) & cond
            if remove.any():
                img[remove] = 0
                changed = True
        if not changed:
            break
    return img.astype(bool)


def trace(mask: np.ndarray) -> List[np.ndarray]:
    """Follow one-pixel-wide lines in a boolean mask. Returns polylines of (x, y) pixels."""
    grid = np.pad(mask.astype(np.uint8), 1)
    kernel = np.ones((3, 3), np.float32)
    degree = cv2.filter2D(grid, cv2.CV_32F, kernel, borderType=cv2.BORDER_CONSTANT) - grid
    ys, xs = np.nonzero(grid)
    deg = degree[ys, xs]
    # Line ends first, so open lines are traced end to end rather than from the middle.
    starts = np.argsort(deg != 1, kind="stable")
    lines = []
    for i in starts:
        y, x = int(ys[i]), int(xs[i])
        if not grid[y, x]:
            continue
        grid[y, x] = 0
        line = [(x, y)]
        dy, dx = 0, 0
        while True:
            best, best_score = None, -9
            for ny, nx in _NEIGHBOURS:
                if grid[y + ny, x + nx]:
                    score = ny * dy + nx * dx - 0.1 * (abs(ny) + abs(nx))
                    if score > best_score:
                        best, best_score = (ny, nx), score
            if best is None:
                break
            dy, dx = best
            y, x = y + dy, x + dx
            grid[y, x] = 0
            line.append((x, y))
        if len(line) > 2:
            # A loop ends next to where it began: close it.
            if len(line) > 8 and abs(line[0][0] - x) <= 1 and abs(line[0][1] - y) <= 1:
                line.append(line[0])
            lines.append(np.array(line, np.float32) - 1.0)  # undo the padding
    return lines


def suggest_mode(data: bytes) -> str:
    """Flat artwork (nearly every pixel is black, white or a strong colour) traces best
    along its centre lines; anything else is treated as a photo."""
    img = _decode(data)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    flat = np.mean((gray < 50) | (gray > 205) | (np.ptp(img, axis=2) > 80))
    return "centerline" if flat > 0.9 else "edges"


def join(lines: List[np.ndarray], tol: float) -> List[np.ndarray]:
    """Stitch traced pieces whose ends nearly touch into longer lines, and close loops.

    Thinning and edge detection break lines at corners and crossings; drawing them as one
    stroke saves the arm a stop and a dark hop at every break."""
    closed = [l for l in lines if np.array_equal(l[0], l[-1])]
    open_ = [l for l in lines if not np.array_equal(l[0], l[-1])]
    if not open_:
        return closed
    starts = np.array([l[0] for l in open_])
    ends = np.array([l[-1] for l in open_])
    free = np.ones(len(open_), bool)
    out = []
    for i in range(len(open_)):
        if not free[i]:
            continue
        free[i] = False
        chain = open_[i]
        for _ in range(2):                      # grow the tail, then flip and grow the other end
            while True:
                d_start = np.where(free, np.linalg.norm(starts - chain[-1], axis=1), np.inf)
                d_end = np.where(free, np.linalg.norm(ends - chain[-1], axis=1), np.inf)
                j = int(np.argmin(np.minimum(d_start, d_end)))
                if min(d_start[j], d_end[j]) > tol:
                    break
                free[j] = False
                chain = np.vstack([chain, open_[j] if d_start[j] <= d_end[j] else open_[j][::-1]])
            chain = chain[::-1]
        if len(chain) > 8 and np.linalg.norm(chain[0] - chain[-1]) <= tol:
            chain = np.vstack([chain, chain[:1]])
        out.append(chain)
    return closed + out


def _stroke_color(img: np.ndarray, line: np.ndarray):
    """Colour of the ink a line follows, pushed to full LED brightness.

    Traced lines often sit on the blurry boundary of the ink, so look at the pixels around
    the line and let the most saturated ones decide."""
    step = max(1, len(line) // 60)
    xs = line[::step, 0].astype(int)[:, None] + np.arange(-3, 4)[None, :]
    ys = line[::step, 1].astype(int)[:, None] + np.arange(-3, 4)[None, :]
    xs = np.clip(xs[:, None, :], 0, img.shape[1] - 1)
    ys = np.clip(ys[:, :, None], 0, img.shape[0] - 1)
    px = img[ys, xs].reshape(-1, 3).astype(float)
    sat = np.ptp(px, axis=1)
    if sat.max() < 40:                     # black, white or grey line
        return (255, 255, 255)
    b, g, r = (px * (sat**2)[:, None]).sum(axis=0) / (sat**2).sum()
    rgb = np.array([r, g, b])
    rgb = rgb - rgb.min() * 0.5
    return tuple(int(v) for v in np.clip(rgb * 255.0 / rgb.max(), 0, 255))


def load_image_strokes(data: bytes, mode: str = "edges", detail: float = 0.5,
                       invert: bool = False) -> List[Stroke]:
    """detail in 0..1: higher keeps weaker edges, shorter lines and finer curves."""
    img = _decode(data)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    detail = float(np.clip(detail, 0.0, 1.0))

    if mode == "edges":
        sigma = 3.0 - 2.3 * detail
        blurred = cv2.GaussianBlur(gray, (0, 0), sigma)
        high = 220.0 - 170.0 * detail
        lines = join(trace(cv2.Canny(blurred, 0.4 * high, high) > 0), 4.0)
        closed_all = False
    else:
        blurred = cv2.GaussianBlur(gray, (0, 0), 1.0)
        kind = cv2.THRESH_BINARY if invert else cv2.THRESH_BINARY_INV
        _, ink = cv2.threshold(blurred, 0, 255, kind + cv2.THRESH_OTSU)
        if mode == "outline":
            contours, _ = cv2.findContours(ink, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
            lines = [c.reshape(-1, 2).astype(np.float32) for c in contours if len(c) > 2]
            lines = [np.vstack([c, c[:1]]) for c in lines]
            closed_all = True
        elif mode == "centerline":
            lines = join(trace(thin(ink > 0)), 10.0)
            closed_all = False
        else:
            raise ValueError("unknown image mode %r" % mode)

    min_len = 40.0 - 34.0 * detail          # pixels
    tolerance = 2.0 - 1.6 * detail          # pixels
    kept = []
    for line in lines:
        seg = np.linalg.norm(np.diff(line, axis=0), axis=1).sum()
        if seg < min_len:
            continue
        closed = closed_all or (len(line) > 3 and np.array_equal(line[0], line[-1]))
        color = _stroke_color(img, line)
        simple = cv2.approxPolyDP(line.reshape(-1, 1, 2), tolerance, False).reshape(-1, 2)
        if len(simple) < 2:
            continue
        kept.append((seg, simple, closed, color))
    if not kept:
        raise ValueError("no lines found in this image; try another trace mode or more detail")
    kept.sort(key=lambda k: -k[0])
    strokes = []
    for _, simple, closed, color in kept[:MAX_STROKES]:
        pts = np.column_stack([np.zeros(len(simple)), simple[:, 0], -simple[:, 1]])
        strokes.append(Stroke(pts.astype(float), color, closed))
    return strokes
