from __future__ import annotations

import argparse
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

try:
    sys.stdout.reconfigure(encoding="utf-8")
except (AttributeError, ValueError):
    pass

from .config import load_config, with_overrides
from . import scene as scene_mod
from .scene import scene_from_args
from .candidates import (Poses, sample_pose_regions, dedup_keys, _clearance_mask,
                         _cone_directions, _tangent_basis, _vertical_fov_ok)
from .visibility import compute_visibility
from . import inspector as insp
from .render_figures import capture

_OUT = Path(__file__).with_name("output") / "praesentation"

C_MESH = (0.86, 0.87, 0.89)
C_FACET = (0.10, 0.25, 0.60)
C_NORMAL = (0.95, 0.45, 0.00)
C_KEEP = (0.13, 0.66, 0.30)
C_DROP = (0.85, 0.20, 0.15)
C_RAW = (0.62, 0.64, 0.70)
C_RAY = (0.55, 0.60, 0.70)
C_SEED = (1.00, 0.45, 0.00)
C_PROJ = (0.05, 0.05, 0.05)
C_CAM = (0.00, 0.45, 0.90)

VIEWS = {
    "box": dict(front=(0.6, -0.8, 0.35), up=(0, 0, 1), zoom=0.52),
    "notched_box": dict(front=(0.78, -0.52, 0.36), up=(0, 0, 1), zoom=0.52),
}
_VIEW_DEFAULT = VIEWS["notched_box"]


def _mesh(scene, color=C_MESH):
    import open3d as o3d
    m = o3d.geometry.TriangleMesh(scene.mesh)
    m.paint_uniform_color(list(color))
    m.compute_vertex_normals()
    return m


def _cloud(pts, colors):
    import open3d as o3d
    pts = np.asarray(pts, float).reshape(-1, 3)
    if not len(pts):
        return None
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
    colors = np.asarray(colors, float)
    if colors.ndim == 1:
        colors = np.tile(colors, (len(pts), 1))
    pc.colors = o3d.utility.Vector3dVector(colors)
    return pc


def _ball(p, r, color):
    import open3d as o3d
    b = o3d.geometry.TriangleMesh.create_sphere(radius=r, resolution=24)
    b.translate(np.asarray(p, float))
    b.paint_uniform_color(list(color))
    b.compute_vertex_normals()
    return b


def _arrows(origins, dirs, length, color, slim=0.06, resolution=8):
    import open3d as o3d
    origins = np.asarray(origins, float).reshape(-1, 3)
    d = np.asarray(dirs, float).reshape(-1, 3)
    if not len(d):
        return None

    r = slim * length
    tmpl = o3d.geometry.TriangleMesh.create_arrow(
        cylinder_radius=r, cone_radius=2.2 * r,
        cylinder_height=0.70 * length, cone_height=0.30 * length,
        resolution=resolution, cylinder_split=1, cone_split=1)
    V0 = np.asarray(tmpl.vertices)
    T0 = np.asarray(tmpl.triangles)

    a = np.column_stack([-d[:, 1], d[:, 0], np.zeros(len(d))])
    c = d[:, 2]
    K = np.zeros((len(d), 3, 3))
    K[:, 0, 1], K[:, 0, 2] = -a[:, 2], a[:, 1]
    K[:, 1, 0], K[:, 1, 2] = a[:, 2], -a[:, 0]
    K[:, 2, 0], K[:, 2, 1] = -a[:, 1], a[:, 0]
    denom = np.maximum(1.0 + c, 1e-12)
    R = np.eye(3) + K + (K @ K) / denom[:, None, None]
    R[1.0 + c < 1e-9] = np.diag([1.0, -1.0, -1.0])

    V = np.einsum("nij,kj->nki", R, V0) + origins[:, None, :]
    T = T0[None] + (np.arange(len(d)) * len(V0))[:, None, None]
    out = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(V.reshape(-1, 3)),
        o3d.utility.Vector3iVector(T.reshape(-1, 3).astype(np.int32)))
    out.paint_uniform_color(list(color))
    out.compute_vertex_normals()
    return out


def _rays(origins, targets, color):
    targets = np.asarray(targets, float).reshape(-1, 3)
    if not len(targets):
        return None
    origins = np.broadcast_to(np.asarray(origins, float).reshape(-1, 3),
                              targets.shape)
    k = len(targets)
    return insp._lineset(np.vstack([origins, targets]),
                         [[i, i + k] for i in range(k)], color)


def _frustum(pos, yaw, pitch, cfg, color):
    pts = insp._frustum_points(np.asarray(pos, float), yaw, pitch, cfg)
    idx = ([[0, 5 + i] for i in range(4)]
           + [[1 + i, 1 + (i + 1) % 4] for i in range(4)]
           + [[5 + i, 5 + (i + 1) % 4] for i in range(4)])
    return insp._lineset(pts, idx, color)


def _anchor(scene, cfg):
    lo, hi = scene.bounds
    m = cfg.d_max
    corners = np.array([[x, y, z] for x in (lo[0] - m, hi[0] + m)
                        for y in (lo[1] - m, hi[1] + m)
                        for z in (lo[2] - m, hi[2] + m)])
    return _cloud(corners, (1.0, 1.0, 1.0))


def _lift(scene, offset_frac=0.006):
    fc = scene.facets
    return fc.centers + fc.normals * (offset_frac * _extent(scene))


def _extent(scene) -> float:
    lo, hi = scene.bounds
    return float((hi - lo).max())


def _crop(img: np.ndarray, frac: float) -> np.ndarray:
    h, w = img.shape[:2]
    ch, cw = int(h * frac), int(w * frac)
    return img[(h - ch) // 2:(h + ch) // 2, (w - cw) // 2:(w + cw) // 2]


def _sub(n: int, k: int) -> np.ndarray:
    if n <= k:
        return np.arange(n)
    return np.unique(np.linspace(0, n - 1, k).astype(np.int64))


class Stages:
    def __init__(self, scene, cfg):
        raw = sample_pose_regions(scene, cfg)
        self.raw = raw
        self.clear = _clearance_mask(raw.positions, scene, cfg.safety_distance)
        self.ground = (raw.positions[:, 2] >= scene.bounds[0][2] + cfg.ground_clearance
                       if cfg.use_ground_plane else np.ones(len(raw), bool))
        self.dist = self.clear & self.ground

        self.pitch = np.clip(raw.pitches, cfg.pitch_min_rad, cfg.pitch_max_rad)
        idx = np.flatnonzero(self.dist)
        self.feasible = np.zeros(len(raw), bool)
        self.feasible[idx] = _vertical_fov_ok(
            raw.positions[idx], raw.yaws[idx], self.pitch[idx],
            scene.facets.centers[raw.seed_facet[idx]], cfg)

        surv = np.flatnonzero(self.feasible)
        keys = dedup_keys(raw.positions[surv], raw.yaws[surv],
                          self.pitch[surv], cfg)
        _, first = np.unique(keys, axis=0, return_index=True)
        self.survivors = surv
        self.reps = surv[np.sort(first)]

    def summary(self) -> str:
        return (f"  roh {len(self.raw):,} -> Abstand {int(self.dist.sum()):,} "
                f"-> Machbarkeit {int(self.feasible.sum()):,} "
                f"-> Dedup {len(self.reps):,}")


def _facet_ps(scene, cfg) -> float:
    return float(np.clip(320.0 * cfg.resolution / _extent(scene), 3.0, 10.0))


def fig_facetten(scene, cfg, st):
    return [_mesh(scene), _cloud(_lift(scene), C_FACET)], _facet_ps(scene, cfg)


def fig_normalen(scene, cfg, st):
    fc = scene.facets
    pts = _lift(scene)
    length = min(1.8 * cfg.resolution, 0.09 * _extent(scene))
    return [_mesh(scene), _cloud(pts, C_FACET),
            _arrows(pts, fc.normals, length, C_NORMAL,
                    slim=0.045)], _facet_ps(scene, cfg)


def _seed_facet(scene, front) -> int:
    fc = scene.facets
    f = np.asarray(front, float) / np.linalg.norm(front)
    cand = np.flatnonzero(fc.normals @ f > 0.5)
    if not len(cand):
        cand = np.arange(len(fc))
    target = fc.centers.mean(axis=0) + f * 0.5 * _extent(scene)
    return int(cand[np.argmin(np.linalg.norm(fc.centers[cand] - target, axis=1))])


def fig_kegel(scene, cfg, st):
    fc = scene.facets
    fi = _seed_facet(scene, VIEWS.get(scene.name, _VIEW_DEFAULT)["front"])
    c, n = fc.centers[fi], fc.normals[fi]

    t1, t2 = _tangent_basis(n[None, :])
    dirs = _cone_directions(n[None, :], t1, t2, cfg)[:, 0, :]
    dists = (np.array([cfg.d_opt]) if cfg.n_distance == 1
             else np.linspace(cfg.d_min, cfg.d_max, cfg.n_distance))
    pos = (c[None, None, :] + dirs[:, None, :] * dists[None, :, None]).reshape(-1, 3)

    return [_mesh(scene), _ball(c, 0.03 * _extent(scene), C_SEED),
            _rays(c, c + dirs * cfg.d_max, C_RAY),
            _cloud(pos, C_KEEP)], 12.0


def _thin(idx, k: int) -> np.ndarray:
    idx = np.asarray(idx)
    return idx[_sub(len(idx), k)]


def fig_dedup(scene, cfg, st):
    pos = st.raw.positions
    drop = _thin(np.setdiff1d(st.survivors, st.reps), 10_000)
    return [_mesh(scene), _cloud(pos[drop], (0.78, 0.80, 0.85)),
            _cloud(pos[st.reps], C_KEEP)], 5.0


def fig_abstand(scene, cfg, st):
    pos = st.raw.positions
    lo, hi = scene.bounds
    grid = insp._make_ground_grid(float(lo[2]), lo, hi,
                                  step=max(0.5, round(_extent(scene) / 10, 1)))
    return [_mesh(scene), grid,
            _cloud(pos[_thin(np.flatnonzero(~st.dist), 40_000)], C_DROP),
            _cloud(pos[_thin(np.flatnonzero(st.dist), 40_000)], C_KEEP)], 3.5


def fig_dualsicht(scene, cfg, st, row=None, n_rays=45):
    poses = Poses(st.raw.positions[st.reps], st.raw.yaws[st.reps],
                  st.pitch[st.reps], np.arange(len(st.reps)),
                  st.raw.seed_facet[st.reps])
    v_dual = compute_visibility(scene, poses, cfg)
    v_single = compute_visibility(scene, poses, replace(cfg, baseline=0.0))
    diff = (v_single.V.astype(np.int8) - v_dual.V.astype(np.int8)).tocsr()
    diff.eliminate_zeros()

    if row is None:
        f = np.asarray(VIEWS.get(scene.name, _VIEW_DEFAULT)["front"], float)
        w = (scene.facets.normals @ (f / np.linalg.norm(f)) > 0.3).astype(np.int32)
        row = int(np.argmax((diff @ w) * (v_dual.V.astype(np.int32) @ w)))
    lost, kept = diff[row].indices, v_dual.V[row].indices
    print(f"  Dualsicht: Pose #{row}, {len(kept)} Facetten dual, "
          f"{len(lost)} nur vom Projektor")

    p = poses.positions[row]
    yaw, pitch = float(poses.yaws[row]), float(poses.pitches[row])
    _, right, _ = insp._pose_frame(yaw, pitch)
    p_cam = p + cfg.baseline * right

    colors = np.tile(np.asarray(C_MESH), (len(scene.facets), 1))
    colors[kept] = C_KEEP
    colors[lost] = C_DROP
    fmesh = insp._make_facet_mesh(scene)
    insp._set_facet_colors(fmesh, colors)

    ctrs = scene.facets.centers
    r = 0.03 * _extent(scene)
    return [fmesh,
            _frustum(p, yaw, pitch, cfg, C_PROJ),
            _frustum(p_cam, yaw, pitch, cfg, C_CAM),
            insp._lineset([p, p_cam], [[0, 1]], C_CAM),
            _rays(p_cam, ctrs[lost[_sub(len(lost), n_rays)]], C_DROP),
            _rays(p, ctrs[kept[_sub(len(kept), n_rays)]], C_KEEP),
            _ball(p, r, C_PROJ), _ball(p_cam, r, C_CAM)], 6.0


FIGURES = {
    1: ("1_facetten", fig_facetten),
    2: ("2_normalen", fig_normalen),
    3: ("3_kegelsampling", fig_kegel),
    4: ("4_dedup", fig_dedup),
    5: ("5_abstand", fig_abstand),
    6: ("6_dualsicht", fig_dualsicht),
}


def render(key: int, scene, cfg, st, args, out_dir: Path) -> None:
    from PIL import Image

    name, builder = FIGURES[key]
    print(f"[{key}] {name} ...")
    geoms, point_size = builder(scene, cfg, st)
    geoms = [g for g in geoms if g is not None] + [_anchor(scene, cfg)]
    lo, hi = scene.bounds
    img = _crop(capture(geoms, width=args.width, height=args.height,
                        point_size=point_size, lookat=0.5 * (lo + hi),
                        **VIEWS.get(scene.name, _VIEW_DEFAULT)), args.crop)

    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{scene.name}_{name}.png"
    Image.fromarray(img).save(path)
    print(f"    -> {path}  ({img.shape[1]}x{img.shape[0]} px)")


def main() -> None:
    ap = argparse.ArgumentParser(
        description="vpp3d — Folien-Abbildungen der Kandidatenkonstruktion")
    ap.add_argument("--scene", default="notched_box", choices=list(scene_mod.SCENES))
    ap.add_argument("--mesh", default=None, help="statt Szene: Mesh/Punktwolke")
    ap.add_argument("--assume-convex", action="store_true")
    ap.add_argument("--resolution", type=float, default=0.08,
                    help="Facetten-Kantenlaenge [m] (fein fuer Folienbilder)")
    ap.add_argument("--baseline", type=float, default=None)
    ap.add_argument("--config", default=None)
    ap.add_argument("--only", type=int, nargs="+", choices=list(FIGURES),
                    default=None, help="nur diese Abbildungen rendern")
    ap.add_argument("--width", type=int, default=3000)
    ap.add_argument("--height", type=int, default=2250)
    ap.add_argument("--crop", type=float, default=0.62,
                    help="mittiger Bildausschnitt (1.0 = gesamtes Bild)")
    ap.add_argument("--out", default=None, help="Zielverzeichnis")
    args = ap.parse_args()

    cfg = with_overrides(load_config(args.config), args)
    scene = scene_from_args(args, cfg)
    print(scene.summary())

    st = Stages(scene, cfg)
    print(st.summary())

    out_dir = Path(args.out) if args.out else _OUT
    for key in (args.only or sorted(FIGURES)):
        render(key, scene, cfg, st, args, out_dir)


if __name__ == "__main__":
    main()
