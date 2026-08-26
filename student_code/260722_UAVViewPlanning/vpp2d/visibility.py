from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np
import scipy.sparse as sp

from .candidates import Poses
from .config import Config
from .scene import Scene


@dataclass
class VisibilityMatrix:
    V: sp.csr_matrix
    stats: dict = field(default_factory=dict)
    runtime: float = 0.0

    @property
    def shape(self) -> tuple[int, int]:
        return self.V.shape

    def coverage_per_facet(self) -> np.ndarray:
        return np.asarray(self.V.sum(axis=0)).ravel()

    def summary(self) -> str:
        cov = self.coverage_per_facet()
        reach = int((cov > 0).sum())
        n = self.V.shape[1]
        return (
            f"Coverage-Matrix V: {self.V.shape[0]} Posen × {n} Facetten, "
            f"{self.V.nnz:,} Sichtbarkeiten\n"
            f"  erreichbare Facetten: {reach}/{n} ({reach / max(1, n):.1%})\n"
            f"  Rechenzeit          : {self.runtime:.2f} s"
        )


def _rays_clear(o: np.ndarray, cs: np.ndarray, edges: np.ndarray) -> np.ndarray:
    o = np.asarray(o, float)
    r = cs - o
    a = edges[:, 0, :]
    e = edges[:, 1, :] - a
    diff = a - o
    denom = r[:, 0:1] * e[:, 1] - r[:, 1:2] * e[:, 0]
    num_t = diff[:, 0] * e[:, 1] - diff[:, 1] * e[:, 0]
    num_s = diff[None, :, 0] * r[:, 1:2] - diff[None, :, 1] * r[:, 0:1]
    with np.errstate(divide="ignore", invalid="ignore"):
        t = num_t[None, :] / denom
        s = num_s / denom
    parallel = np.abs(denom) < 1e-12
    hit = (~parallel) & (t > 1e-6) & (t < 1 - 1e-4) & (s > -1e-9) & (s < 1 + 1e-9)
    return ~hit.any(axis=1)


def compute_visibility(scene: Scene, poses: Poses, cfg: Config) -> VisibilityMatrix:
    t0 = time.perf_counter()
    C = scene.facets.centers
    Nn = scene.facets.normals
    edges = scene.edges
    cos_fov = np.cos(cfg.fov_rad / 2.0)
    cos_theta = np.cos(cfg.theta_max_rad)

    rows, cols = [], []
    stat = {"dist": 0, "incidence": 0, "fov": 0, "occlusion_single": 0, "dual": 0}

    for m in range(len(poses)):
        p = poses.positions[m]
        yaw = poses.yaws[m]
        u = np.array([np.cos(yaw), np.sin(yaw)])
        right = np.array([u[1], -u[0]])
        p_cam = p + cfg.baseline * right

        r = C - p
        dist = np.linalg.norm(r, axis=1)
        ok = (dist >= cfg.d_min) & (dist <= cfg.d_max)
        stat["dist"] += int(ok.sum())
        if not ok.any():
            continue

        rn = r / dist[:, None]
        cos_inc = -(Nn * rn).sum(axis=1)
        ok &= cos_inc >= cos_theta
        stat["incidence"] += int(ok.sum())
        if not ok.any():
            continue

        cos_view = (rn * u).sum(axis=1)
        ok &= cos_view >= cos_fov
        r_cam = C - p_cam
        dist_cam = np.linalg.norm(r_cam, axis=1)
        cos_view_cam = (r_cam * u).sum(axis=1) / np.maximum(dist_cam, 1e-12)
        ok &= cos_view_cam >= cos_fov
        stat["fov"] += int(ok.sum())
        if not ok.any():
            continue

        idx = np.flatnonzero(ok)
        clear_p = _rays_clear(p, C[idx], edges)
        stat["occlusion_single"] += int(clear_p.sum())
        idx = idx[clear_p]
        if idx.size == 0:
            continue
        if cfg.baseline > 0:
            clear_c = _rays_clear(p_cam, C[idx], edges)
            idx = idx[clear_c]
        stat["dual"] += int(idx.size)
        if idx.size == 0:
            continue

        rows.append(np.full(idx.size, m, dtype=np.int64))
        cols.append(idx)

    if rows:
        rows = np.concatenate(rows)
        cols = np.concatenate(cols)
    else:
        rows = cols = np.empty(0, dtype=np.int64)
    V = sp.csr_matrix(
        (np.ones(len(rows), dtype=bool), (rows, cols)),
        shape=(len(poses), len(scene.facets)),
    )
    return VisibilityMatrix(V=V, stats=stat, runtime=time.perf_counter() - t0)
