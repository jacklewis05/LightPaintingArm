"""3D model -> wireframe strokes: keep the sharp and boundary edges, chain them into long strokes."""
from __future__ import annotations

import io
from collections import defaultdict
from typing import List

import numpy as np
import trimesh

from .paths import Stroke

MESH_TYPES = {"stl", "obj", "ply", "off", "glb"}
MAX_EDGES = 20000


def feature_edges(mesh: trimesh.Trimesh, angle_deg: float) -> np.ndarray:
    """Edges (M, 2) whose two faces meet at more than angle_deg, plus open boundary edges.

    angle_deg <= 0 keeps every edge (full triangle wireframe)."""
    if angle_deg <= 0:
        return mesh.edges_unique
    parts = []
    if len(mesh.face_adjacency):
        sharp = mesh.face_adjacency_angles > np.radians(angle_deg)
        parts.append(np.sort(mesh.face_adjacency_edges[sharp], axis=1))
    boundary = trimesh.grouping.group_rows(mesh.edges_sorted, require_count=1)
    if len(boundary):
        parts.append(mesh.edges_sorted[boundary])
    if not parts:
        return np.zeros((0, 2), int)
    return np.unique(np.vstack(parts), axis=0)


def chain_edges(vertices: np.ndarray, edges: np.ndarray) -> List[np.ndarray]:
    """Join edges into trails of vertex indices, always continuing as straight as possible.

    A trail ends when it returns to its start (a closed loop) or runs out of unused edges,
    so e.g. an octahedron comes out as its three squares."""
    adj = defaultdict(list)
    for ei, (a, b) in enumerate(edges):
        adj[int(a)].append((int(b), ei))
        adj[int(b)].append((int(a), ei))
    used = np.zeros(len(edges), bool)
    remaining = {v: len(n) for v, n in adj.items()}

    def unit(a, b):
        d = vertices[b] - vertices[a]
        n = np.linalg.norm(d)
        return d / n if n > 0 else d

    # Trails that start at odd-degree vertices use the fewest pen-lifts.
    order = sorted(adj, key=lambda v: (len(adj[v]) % 2 == 0, v))
    trails = []
    for start in order:
        while remaining[start] > 0:
            trail = [start]
            cur, heading = start, None
            while True:
                best, best_score = None, -2.0
                for nxt, ei in adj[cur]:
                    if used[ei]:
                        continue
                    score = 1.0 if heading is None else float(heading @ unit(cur, nxt))
                    if score > best_score:
                        best, best_score = (nxt, ei), score
                if best is None:
                    break
                nxt, ei = best
                used[ei] = True
                remaining[cur] -= 1
                remaining[nxt] -= 1
                heading = unit(cur, nxt)
                trail.append(nxt)
                cur = nxt
                if cur == start:
                    break
            trails.append(np.array(trail))
    return trails


def load_mesh_strokes(data: bytes, ext: str, feature_angle: float = 20.0) -> List[Stroke]:
    mesh = trimesh.load(io.BytesIO(data), file_type=ext, force="mesh")
    if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
        raise ValueError("this file has no triangles to take edges from")
    mesh.merge_vertices()
    edges = feature_edges(mesh, feature_angle)
    if len(edges) == 0:
        raise ValueError(
            "no edges sharper than %.0f degrees; lower the edge angle (0 draws every triangle edge)"
            % feature_angle
        )
    if len(edges) > MAX_EDGES:
        raise ValueError(
            "%d edges is too many to light-paint; raise the edge angle or simplify the model"
            % len(edges)
        )
    verts = np.asarray(mesh.vertices, float)
    strokes = []
    for trail in chain_edges(verts, edges):
        closed = len(trail) > 2 and trail[0] == trail[-1]
        strokes.append(Stroke(verts[trail], closed=closed))
    return strokes
