"""Run with:  .venv/bin/python tests/test_core.py"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from lightpaint import planner  # noqa: E402
from lightpaint.config import ArmConfig  # noqa: E402
from lightpaint.image_paths import load_image_strokes  # noqa: E402
from lightpaint.kinematics import fk, ik  # noqa: E402
from lightpaint.mesh_paths import load_mesh_strokes  # noqa: E402
from lightpaint.paths import Stroke, colorize, normalize, place  # noqa: E402

EX = Path(__file__).resolve().parent.parent / "examples"
cfg = ArmConfig.load()


def test_ik_inverts_fk():
    rng = np.random.default_rng(1)
    q = rng.uniform(cfg.q_min, cfg.q_max, size=(2000, 3))
    p = fk(cfg, q)
    q2, ok = ik(cfg, p)
    ok &= p[:, 2] >= cfg.min_z
    assert ok.sum() > 1000
    assert np.allclose(fk(cfg, q2[ok]), p[ok], atol=1e-6)
    # When the tip is in front of the base axis the solution is unique; behind it the
    # arm could also turn the base half way round, so only the position has to match.
    qr = np.radians(q)
    front = ok & (cfg.l1 * np.cos(qr[:, 1]) + cfg.l2 * np.cos(qr[:, 1] + qr[:, 2]) > 1.0)
    assert np.allclose(q2[front], q[front], atol=1e-6)


def test_ik_with_offsets():
    raw = dict(cfg.raw, geometry_mm=dict(cfg.raw["geometry_mm"], lateral_offset=35, shoulder_forward=20))
    c = ArmConfig(raw)
    q = np.array([[30.0, 70.0, -80.0], [-100.0, 60.0, -40.0]])
    q2, ok = ik(c, fk(c, q))
    assert ok.all() and np.allclose(q2, q, atol=1e-6)


def test_steps_round_trip():
    q = np.array([12.5, 80.0, -45.0])
    assert np.allclose(cfg.steps_to_deg(cfg.deg_to_steps(q)), q, atol=1 / cfg.steps_per_deg)
    assert (cfg.deg_to_steps(cfg.q_home) == 0).all()


def test_octahedron_is_three_squares():
    strokes = load_mesh_strokes((EX / "octahedron.obj").read_bytes(), "obj", 20)
    assert len(strokes) == 3
    assert all(s.closed and len(s.pts) == 5 for s in strokes)


def test_cube_has_twelve_edges():
    strokes = load_mesh_strokes((EX / "cube.obj").read_bytes(), "obj", 20)
    assert sum(len(s.pts) - 1 for s in strokes) == 12      # face diagonals are not drawn
    every = load_mesh_strokes((EX / "cube.obj").read_bytes(), "obj", 0)
    assert sum(len(s.pts) - 1 for s in every) == 18


def test_obj_line_elements_are_drawn_as_written():
    obj = b"v 0 0 0\nv 1 0 0\nv 1 1 0\nl 1 2 3 1\nl -1 -3\nf 1 2 3\n"
    strokes = load_mesh_strokes(obj, "obj", 20)
    assert [len(s.pts) for s in strokes] == [4, 2]
    assert strokes[0].closed and not strokes[1].closed
    assert np.allclose(strokes[1].pts, [[1, 1, 0], [0, 0, 0]])


def test_rotunda_example_is_a_line_drawing():
    strokes = load_mesh_strokes((EX / "rotunda.obj").read_bytes(), "obj", 20)
    assert 20 <= len(strokes) <= 60
    pts = np.vstack([s.pts for s in strokes])
    assert pts[:, 2].max() > 14 and pts[:, 0].max() > 17        # dome on top, portico toward +X


def test_image_modes_find_the_star():
    data = (EX / "star.png").read_bytes()
    for mode in ("edges", "outline", "centerline"):
        strokes = load_image_strokes(data, mode, 0.5)
        assert 1 <= len(strokes) <= 12, (mode, len(strokes))
        assert all((s.pts[:, 0] == 0).all() for s in strokes)
    centre = load_image_strokes(data, "centerline", 0.5)
    assert any(s.rgb[0] > 200 and s.rgb[2] < 120 for s in centre)   # the red star keeps its colour


def _plan_octahedron(**kw):
    strokes, _ = normalize(load_mesh_strokes((EX / "octahedron.obj").read_bytes(), "obj", 20))
    pl = planner.autofit(cfg, strokes, {"rx": 20, "ry": 30, "rz": 0})
    return planner.plan(cfg, place(colorize(strokes, "palette", None), pl), **kw), pl


def test_plan_respects_limits():
    job, pl = _plan_octahedron(draw_speed=60)
    assert pl["size"] > 50
    assert job.stats["unreachable_mm"] == 0 and job.stats["strokes"] == 3
    assert np.all(job.q >= cfg.q_min - 1e-6) and np.all(job.q <= cfg.q_max + 1e-6)
    vel = np.abs(np.diff(job.q, axis=0)) / cfg.dt
    assert np.all(vel <= cfg.v_max * 1.02), vel.max(axis=0)
    # speed may change by the acceleration limit, plus one allowed instant jump at a corner
    dv = np.abs(np.diff(job.q, n=2, axis=0)) / cfg.dt
    assert np.all(dv <= cfg.a_max * cfg.dt * 1.5 + cfg.v_jump * 1.05), dv.max(axis=0)
    tip_speed = np.linalg.norm(np.diff(job.tip, axis=0), axis=1) / cfg.dt
    assert tip_speed[job.path[1:] >= 0].max() <= 60 * 1.02
    assert np.allclose(job.q[0], cfg.q_home) and np.allclose(job.q[-1], cfg.q_home)
    assert (job.rgb[job.path < 0] == 0).all()               # dark while travelling


def test_drawn_lines_are_straight():
    job, _ = _plan_octahedron(draw_speed=40)
    lit = job.path == 0
    pts = job.tip[lit]
    # every lit sample of the first square lies in that square's plane
    centred = pts - pts.mean(axis=0)
    assert np.linalg.svd(centred, compute_uv=False)[2] < 0.5


def test_even_exposure_dims_slow_parts():
    job, _ = _plan_octahedron(draw_speed=40, even_exposure=True)
    lit = np.flatnonzero(job.path == 0)
    speed = np.linalg.norm(job.tip[lit] - job.tip[lit - 1], axis=1) / cfg.dt
    level = job.rgb[lit].max(axis=1) / 255.0
    assert np.allclose(level, np.clip(speed / 40, 0, 1), atol=0.02)
    flat, _ = _plan_octahedron(draw_speed=40, even_exposure=False)
    assert (flat.rgb[flat.path >= 0].max(axis=1) == 255).all()


def test_unreachable_parts_are_reported():
    far = Stroke(np.array([[300.0, 0, 200], [900.0, 0, 200]]))
    job = planner.plan(cfg, [far])
    assert job.stats["unreachable_mm"] > 300 and job.stats["draw_mm"] > 100
    assert np.linalg.norm(job.tip[job.path >= 0], axis=1).max() < 700


def test_animation_frames_spin_about_the_centre():
    strokes, _ = normalize(load_mesh_strokes((EX / "octahedron.obj").read_bytes(), "obj", 20))
    spins = [("z", 15.0 * i) for i in range(24)]
    pl = planner.autofit(cfg, strokes, {"rx": 20, "ry": 0, "rz": 0}, spins=spins)
    centre = np.array([pl["x"], pl["y"], pl["z"]])
    first = np.vstack([s.pts for s in place(strokes, pl)])
    for axis, deg in spins[::5]:
        frame = place(strokes, dict(pl, spin_axis=axis, spin_deg=deg))
        pts = np.vstack([s.pts for s in frame])
        # turning in place: every point keeps its distance from the centre
        assert np.allclose(np.linalg.norm(pts - centre, axis=1), np.linalg.norm(first - centre, axis=1))
        assert planner.plan(cfg, frame).stats["unreachable_mm"] == 0                   # every frame fits
    quarter = np.vstack([s.pts for s in place(strokes, dict(pl, spin_axis="z", spin_deg=90))])
    assert np.allclose(quarter[:, 0] - centre[0], -(first[:, 1] - centre[1]), atol=1e-6)


def test_travel_profile():
    a, b = np.array([0.0, 90, -90]), np.array([100.0, 40, -30])
    s = planner.travel(cfg, a, b)
    assert np.allclose(s[-1], b)
    full = np.vstack([a, s])
    assert np.all(np.abs(np.diff(full, axis=0)) / cfg.dt <= cfg.v_max * 1.001)
    assert np.all(np.abs(np.diff(full, n=2, axis=0)) / cfg.dt**2 <= cfg.a_max * 1.05)
    assert len(planner.travel(cfg, a, a)) == 0


def test_status_line_with_encoders():
    from lightpaint.device import SerialDevice
    dev = SerialDevice.__new__(SerialDevice)        # parse only; no serial port needed
    dev.log, dev.error = [], None
    dev._handle("st 100 -200 300 7 2 7 R 255 0 10 5 0 -64 0 1")
    st = dev._status
    assert st["steps"] == [100.0, -200.0, 300.0] and st["done"] == 7 and st["homed"]
    assert st["enc"] == [True, False, True] and st["slip"] == [0.0, -64.0, 0.0] and st["motors"]
    dev._handle("err not homed")
    assert dev.error == "not homed"


def test_sim_resync_adopts_encoder_position():
    from lightpaint.device import SimDevice
    dev = SimDevice(cfg)
    dev.fake_slip(1, -64)
    assert dev.status()["slip"][1] == -64
    dev.resync()
    st = dev.status()
    assert st["steps"][1] == -64 and st["slip"] == [0.0, 0.0, 0.0]


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok ", name)
