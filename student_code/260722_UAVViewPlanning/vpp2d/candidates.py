from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import cKDTree

from .config import Config
from .scene import Scene


@dataclass
class Poses:
    positions: np.ndarray
    yaws: np.ndarray
    ids: np.ndarray
    seed_facet: np.ndarray = field(default_factory=lambda: np.empty(0, np.int64))

    def __len__(self) -> int:
        return len(self.ids)

    @property
    def view_dirs(self) -> np.ndarray:
        return np.column_stack([np.cos(self.yaws), np.sin(self.yaws)])


def _rotate(vec: np.ndarray, angles: np.ndarray) -> np.ndarray:
    c, s = np.cos(angles), np.sin(angles)
    return np.column_stack([c * vec[0] - s * vec[1], s * vec[0] + c * vec[1]])


def sample_pose_regions(scene: Scene, cfg: Config) -> Poses:
    incs = np.linspace(-cfg.theta_max_rad, cfg.theta_max_rad, cfg.n_incidence)
    dists = (np.array([cfg.d_opt]) if cfg.n_distance == 1
             else np.linspace(cfg.d_min, cfg.d_max, cfg.n_distance))

    pos_list, yaw_list, seed_list = [], [], []
    for fid, (c, n) in enumerate(zip(scene.facets.centers, scene.facets.normals)):
        dirs = _rotate(n, incs)
        for d in dists:
            p = c[None, :] + dirs * d
            to_facet = c[None, :] - p
            yaw = np.arctan2(to_facet[:, 1], to_facet[:, 0])
            pos_list.append(p)
            yaw_list.append(yaw)
            seed_list.append(np.full(len(p), fid, dtype=np.int64))

    positions = np.vstack(pos_list)
    yaws = np.concatenate(yaw_list)
    seed = np.concatenate(seed_list)
    return Poses(positions, yaws, np.arange(len(positions), dtype=np.int64), seed)


def _clearance_mask(positions: np.ndarray, scene: Scene, safety: float) -> np.ndarray:
    tree = cKDTree(scene.facets.centers)
    dist, _ = tree.query(positions, k=1)
    return dist >= safety


def project_and_dedup(raw: Poses, scene: Scene, cfg: Config) -> tuple[Poses, dict]:
    keep = _clearance_mask(raw.positions, scene, cfg.safety_distance)
    pos = raw.positions[keep]
    yaw = raw.yaws[keep]
    seed = raw.seed_facet[keep]
    n_after_clear = len(pos)

    yaw_mod = np.mod(yaw, 2 * np.pi)
    yaw_bins = np.floor(yaw_mod / cfg.yaw_bin_rad).astype(np.int64)

    origin = pos.min(axis=0)
    ix = np.floor((pos[:, 0] - origin[0]) / cfg.voxel).astype(np.int64)
    iy = np.floor((pos[:, 1] - origin[1]) / cfg.voxel).astype(np.int64)

    keys = np.column_stack([ix, iy, yaw_bins])
    _, first_idx = np.unique(keys, axis=0, return_index=True)
    first_idx = np.sort(first_idx)

    poses = Poses(
        positions=pos[first_idx],
        yaws=yaw[first_idx],
        ids=np.arange(len(first_idx), dtype=np.int64),
        seed_facet=seed[first_idx],
    )
    stats = {
        "n_raw": len(raw),
        "n_after_clearance": n_after_clear,
        "n_after_dedup": len(poses),
        "dedup_ratio": len(poses) / max(1, n_after_clear),
    }
    return poses, stats
