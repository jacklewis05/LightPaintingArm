"""Regenerates rotunda.obj and uva_logo.png:  .venv/bin/python examples/make_examples.py

Both are simplified drawings made for light painting (few, long lines), not accurate
models or official artwork.
"""
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent


# ------------------------------------------------------------------ rotunda.obj
# A line drawing: the OBJ holds only vertices and "l" line elements, and the program
# draws each line element as one stroke. Z is up; the portico faces +X, toward a camera
# standing in front of the arm.

def ring(radius, z, n=48):
    a = np.linspace(0, 2 * np.pi, n + 1)
    return np.column_stack([radius * np.cos(a), radius * np.sin(a), np.full(n + 1, z)])


def rotunda_lines():
    lines = []
    drum_r, drum_top = 10.0, 9.0
    dome_z, dome_h, oculus = 9.8, 5.6, np.radians(76)

    # Drum: base and top rings, with uprights between them so it reads as a cylinder.
    lines += [ring(drum_r, 0.0), ring(drum_r, drum_top), ring(drum_r + 0.5, dome_z)]
    for a in np.radians(np.arange(22.5, 360, 45)):
        if abs(np.sin(a)) * drum_r < 8.0 and np.cos(a) > 0:
            continue                                   # hidden behind the portico
        lines.append([(drum_r * np.cos(a), drum_r * np.sin(a), z) for z in (0.0, drum_top)])
    # Dome: ribs from its base up to the oculus ring.
    t = np.linspace(0, oculus, 12)
    for a in np.radians(np.arange(0, 360, 45)):
        r = 9.8 * np.cos(t)
        lines.append(np.column_stack([r * np.cos(a), r * np.sin(a), dome_z + dome_h * np.sin(t)]))
    lines.append(ring(9.8 * np.cos(oculus), dome_z + dome_h * np.sin(oculus), 24))

    # Portico: six columns, the entablature across them, and the pediment.
    front, half, col_top, eave, apex = 16.0, 8.0, 7.0, 8.1, 11.2
    back = np.sqrt(drum_r**2 - half**2)                # where the side walls meet the drum
    for y in np.linspace(-7.0, 7.0, 6):
        lines.append([(front, y, 0.0), (front, y, col_top)])
    lines.append([(front, -half, col_top), (front, half, col_top), (front, half, eave),
                  (front, -half, eave), (front, -half, col_top)])
    lines.append([(front, -half, eave), (front, 0.0, apex), (front, half, eave)])
    for y in (-half, half):                            # eaves and floor edges back to the drum
        lines.append([(front, y, eave), (back, y, eave)])
        lines.append([(front, y, 0.0), (back, y, 0.0)])
    lines.append([(front, 0.0, apex), (drum_r, 0.0, apex)])          # roof ridge
    # Steps: the floor edge and two treads, each a little wider and lower.
    lines.append([(front, -half, 0.0), (front, half, 0.0)])
    for out, z in ((1.6, -0.7), (3.2, -1.4)):
        lines.append([(front, -half, z), (front + out, -half, z), (front + out, half, z), (front, half, z)])
    return [np.asarray(line, float) for line in lines]


def make_rotunda():
    out = ["# Simplified UVA Rotunda as a line drawing for light painting.",
           "# Z is up and the portico faces +X. Each 'l' element is drawn as one stroke."]
    count = 0
    elements = []
    for line in rotunda_lines():
        closed = len(line) > 2 and np.allclose(line[0], line[-1])
        pts = line[:-1] if closed else line
        out += ["v %.3f %.3f %.3f" % tuple(p) for p in pts]
        idx = list(range(count + 1, count + len(pts) + 1))
        elements.append("l " + " ".join(str(i) for i in idx + idx[:1] * closed))
        count += len(pts)
    (HERE / "rotunda.obj").write_text("\n".join(out + elements) + "\n")


# ------------------------------------------------------------------ uva_logo.png
# A V above two crossed sabres, in thick single lines: "Centre lines" tracing turns each
# into one stroke. Where one line passes behind another it is drawn with a gap, so the
# tracer sees separate clean lines instead of a tangle at the crossing.

ORANGE = (0, 114, 229)      # BGR, UVA orange
BLUE = (200, 80, 40)        # a brighter stand-in for UVA navy, which an LED cannot show
WHITE = (255, 255, 255)


def curve(p0, p1, p2, n=80):
    t = np.linspace(0, 1, n)[:, None]
    return (1 - t) ** 2 * np.array(p0, float) + 2 * t * (1 - t) * np.array(p1, float) + t**2 * np.array(p2, float)


def make_logo():
    img = np.full((800, 800, 3), 255, np.uint8)

    def stroke(pts, colour, width=12, over=True):
        pts = np.asarray(pts, np.int32)
        if over:    # clear a margin first, so whatever is underneath stops short of this line
            cv2.polylines(img, [pts], False, WHITE, width + 36, cv2.LINE_AA)
        cv2.polylines(img, [pts], False, colour, width, cv2.LINE_AA)

    for flip in (1, -1):                               # the two sabres mirror each other
        mirror = lambda pts: [(400 + flip * (x - 400), y) for x, y in pts]
        stroke(mirror(curve((165, 715), (420, 640), (690, 330))), BLUE)        # blade, hilt to tip
        stroke(mirror(curve((205, 640), (255, 690), (240, 750), 20)), BLUE)    # guard
    stroke([(240, 130), (400, 520), (560, 130)], ORANGE, 16, over=False)       # the V
    stroke([(190, 130), (290, 130)], ORANGE, 16, over=False)                   # its serifs
    stroke([(510, 130), (610, 130)], ORANGE, 16, over=False)
    cv2.imwrite(str(HERE / "uva_logo.png"), img)


if __name__ == "__main__":
    make_rotunda()
    make_logo()
    print("wrote rotunda.obj and uva_logo.png")
