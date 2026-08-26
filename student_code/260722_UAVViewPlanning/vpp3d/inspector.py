from __future__ import annotations

import argparse
import colorsys
import math
import sys
from pathlib import Path

import numpy as np

try:
    sys.stdout.reconfigure(encoding="utf-8")
except (AttributeError, ValueError):
    pass

from .config import load_config, with_overrides
from .scene import scene_from_args
from .candidates import sample_pose_regions, project_and_dedup
from .visibility import compute_visibility
from .setcover import greedy_set_cover, ilp_set_cover, connected_set_cover_ilp
from .erosion import erode_footprints, cover_residual
from .overlap import ensure_connected
from .refine import refine_selected_poses
from .tracking import plan_tracking
from .sequencing import sequence_route
from .runlog import RunLog
from .run import add_common_args

_OUT = Path(__file__).with_name("output")

_KEY_N, _KEY_B = 78, 66
_KEY_RIGHT, _KEY_LEFT = 262, 263
_KEY_C, _KEY_P, _KEY_O, _KEY_M = 67, 80, 79, 77
_KEY_1, _KEY_2, _KEY_3, _KEY_4 = 49, 50, 51, 52
_KEY_S, _KEY_Q, _KEY_ESC = 83, 81, 256

_C_VIS      = np.array([0.18, 0.76, 0.36])
_C_CORE     = np.array([0.00, 0.42, 0.18])
_C_PLAN_COV = np.array([0.60, 0.88, 0.65])
_C_OPEN     = np.array([0.85, 0.22, 0.15])
_C_UNREACH  = np.array([0.42, 0.42, 0.42])
_C_BG       = np.array([0.78, 0.78, 0.78])
_C_CUR      = np.array([1.00, 0.50, 0.00])
_C_OTHER    = np.array([0.30, 0.30, 0.30])
_C_CAND     = np.array([0.28, 0.60, 0.90])
_C_PLAN_PT  = np.array([0.10, 0.10, 0.10])
_C_FRUSTUM  = np.array([0.00, 0.40, 1.00])
_C_ARROW    = np.array([0.10, 0.10, 0.10])
_C_ROUTE    = np.array([1.00, 0.55, 0.00])
_C_OVERLAP  = np.array([1.00, 0.90, 0.00])
_C_TRACKER  = np.array([0.00, 0.80, 0.80])

_FRUSTUM_LINES = np.array([
    [0, 1], [0, 2], [0, 3], [0, 4],
    [1, 2], [2, 3], [3, 4], [4, 1],
    [5, 6], [6, 7], [7, 8], [8, 5],
    [1, 5], [2, 6], [3, 7], [4, 8],
], dtype=np.int32)


def _distinct_colors(n: int) -> np.ndarray:
    golden = 0.6180339887498949
    cols = np.empty((max(1, n), 3))
    for i in range(max(1, n)):
        h = (i * golden) % 1.0
        s = (0.90, 0.60, 0.75)[i % 3]
        v = (0.95, 0.75, 0.85, 0.65)[i % 4]
        cols[i] = colorsys.hsv_to_rgb(h, s, v)
    return cols


def _lineset(pts, lines, color=None):
    import open3d as o3d
    ls = o3d.geometry.LineSet(
        o3d.utility.Vector3dVector(np.asarray(pts, dtype=float)),
        o3d.utility.Vector2iVector(np.asarray(lines, dtype=np.int32)),
    )
    if color is not None:
        ls.paint_uniform_color(list(color))
    return ls


def _set_points(ls, pts) -> None:
    import open3d as o3d
    ls.points = o3d.utility.Vector3dVector(np.asarray(pts, dtype=float))


def _pose_frame(yaw: float, pitch: float):
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    fwd = np.array([cp * cy, cp * sy, sp])
    right = np.array([-sy, cy, 0.0])
    return fwd, right, np.cross(right, fwd)


def _rect(ctr: np.ndarray, a: np.ndarray, b: np.ndarray) -> list[np.ndarray]:
    return [ctr + a + b, ctr - a + b, ctr - a - b, ctr + a - b]


def _make_facet_mesh(scene):
    import open3d as o3d
    V, T = scene.facet_verts, scene.facet_tris
    mesh = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(V[T].reshape(-1, 3)),
        o3d.utility.Vector3iVector(np.arange(len(T) * 3, dtype=np.int32).reshape(-1, 3)),
    )
    mesh.compute_vertex_normals()
    return mesh


def _set_facet_colors(mesh, colors: np.ndarray) -> None:
    import open3d as o3d
    mesh.vertex_colors = o3d.utility.Vector3dVector(np.repeat(colors, 3, axis=0))


def _frustum_points(pos: np.ndarray, yaw: float, pitch: float, cfg) -> np.ndarray:
    fwd, right, up = _pose_frame(yaw, pitch)
    th = math.tan(cfg.fov_h_rad / 2)
    tv = math.tan(cfg.fov_v_rad / 2)
    return np.vstack([pos.reshape(1, 3)]
                     + [_rect(pos + d * fwd, d * th * right, d * tv * up)
                        for d in (cfg.d_min, cfg.d_max)])


def _make_ground_grid(z_ground: float, lo: np.ndarray, hi: np.ndarray,
                      step: float = 1.0):
    margin = max(step, 0.5)
    x0, x1 = lo[0] - margin, hi[0] + margin
    y0, y1 = lo[1] - margin, hi[1] + margin
    pts: list[list[float]] = []
    for y in np.arange(y0, y1 + 1e-9, step):
        pts += [[x0, y, z_ground], [x1, y, z_ground]]
    for x in np.arange(x0, x1 + 1e-9, step):
        pts += [[x, y0, z_ground], [x, y1, z_ground]]
    lines = [[i, i + 1] for i in range(0, len(pts), 2)]
    return _lineset(pts, lines, (0.60, 0.60, 0.60))


def _make_camera_pyramids(poses, selected_rows: np.ndarray,
                          palette: np.ndarray, cfg):
    import open3d as o3d
    depth = 0.18 * cfg.d_opt
    th = np.tan(cfg.fov_h_rad / 2) * depth
    tv = np.tan(cfg.fov_v_rad / 2) * depth

    pts: list[np.ndarray] = []
    idx: list[list[int]] = []
    col: list[list[float]] = []
    for k, row in enumerate(selected_rows):
        pos = poses.positions[row]
        fwd, right, up = _pose_frame(float(poses.yaws[row]),
                                     float(poses.pitches[row]))
        base = len(pts)
        pts.append(pos)
        pts.extend(_rect(pos + depth * fwd, th * right, tv * up))
        c = palette[k].tolist()
        for i in range(4):
            idx.append([base, base + 1 + i]); col.append(c)
        for i in range(4):
            idx.append([base + 1 + i, base + 1 + (i + 1) % 4]); col.append(c)

    ls = _lineset(pts, idx)
    ls.colors = o3d.utility.Vector3dVector(np.array(col, dtype=float))
    return ls


def _make_tracking_geom(tracking, plan_pos: np.ndarray):
    import open3d as o3d

    if tracking is None or len(tracking.stations) == 0:
        return None, None

    pcd = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(tracking.stations))
    pcd.paint_uniform_color(_C_TRACKER.tolist())

    pts: list[np.ndarray] = []
    idx: list[list[int]] = []
    for i, st_idx in enumerate(tracking.assignment):
        if st_idx < 0 or i >= len(plan_pos):
            continue
        pts += [tracking.stations[st_idx], plan_pos[i]]
        idx.append([len(pts) - 2, len(pts) - 1])

    if not idx:
        return pcd, None
    return pcd, _lineset(pts, idx, _C_TRACKER)


def _overlay_core(colors: np.ndarray, V_eroded, row: int) -> None:
    if V_eroded is None:
        return
    core = V_eroded.getrow(row).indices
    if len(core):
        colors[core] = _C_CORE


def _colors_candidates(vis, coverable: np.ndarray, cur_row: int,
                       V_eroded=None) -> np.ndarray:
    colors = np.tile(_C_BG, (len(coverable), 1))
    colors[~coverable] = _C_UNREACH
    visible = vis.V.getrow(cur_row).indices
    if len(visible):
        colors[visible] = _C_VIS
    _overlay_core(colors, V_eroded, cur_row)
    return colors


def _colors_plan(vis, coverable: np.ndarray, selected_rows: np.ndarray,
                 cur_step: int, V_eroded=None) -> np.ndarray:
    plan_covered = np.asarray(vis.V[selected_rows].sum(axis=0)).ravel() > 0
    colors = np.tile(_C_UNREACH, (len(coverable), 1))
    colors[coverable & ~plan_covered] = _C_OPEN
    colors[coverable & plan_covered]  = _C_PLAN_COV
    visible = vis.V.getrow(selected_rows[cur_step]).indices
    if len(visible):
        colors[visible] = _C_VIS
    _overlay_core(colors, V_eroded, selected_rows[cur_step])
    return colors


def _colors_overview(vis, coverable: np.ndarray,
                     selected_rows: np.ndarray) -> np.ndarray:
    plan_covered = np.asarray(vis.V[selected_rows].sum(axis=0)).ravel() > 0
    colors = np.tile(_C_UNREACH, (len(coverable), 1))
    colors[coverable & ~plan_covered] = _C_OPEN
    colors[coverable & plan_covered]  = np.array([0.19, 0.64, 0.33])
    return colors


def _colors_mosaic(vis, coverable: np.ndarray, selected_rows: np.ndarray,
                   palette: np.ndarray,
                   overlap_facets: np.ndarray | None = None) -> np.ndarray:
    V_sel = vis.V[selected_rows].tocsr()
    coverage_count = np.asarray(V_sel.sum(axis=0)).ravel().astype(int)

    colors = np.tile(_C_UNREACH, (len(coverable), 1))
    colors[coverable & (coverage_count == 0)] = _C_OPEN

    coo = V_sel.tocoo()
    if coo.nnz:
        order = np.lexsort((coo.row, coo.col))
        col_s, row_s = coo.col[order], coo.row[order]
        first = np.ones(len(col_s), dtype=bool)
        first[1:] = col_s[1:] != col_s[:-1]
        colors[col_s[first]] = palette[row_s[first]]

    if overlap_facets is not None and len(overlap_facets):
        colors[overlap_facets] = _C_OVERLAP
    return colors


def run_inspector(scene, poses, vis, selected_rows: np.ndarray,
                  route, cfg, overlap=None, tracking=None,
                  V_eroded=None) -> None:
    import open3d as o3d

    coverable = vis.coverage_per_facet() > 0
    fc = scene.facets
    n_cand = len(poses)
    n_plan = len(selected_rows)
    plan_pos = poses.positions[selected_rows]
    palette  = _distinct_colors(n_plan)

    extent   = float(np.ptp(fc.centers, axis=0).max())
    r_sphere = max(0.04, 0.016 * extent)
    arrow_len = 0.70 * cfg.d_opt

    overlap_ids = overlap.overlap_facets if overlap is not None else None

    lo, hi = scene.bounds
    ls_ground = _make_ground_grid(float(lo[2]), lo, hi,
                                  step=max(0.5, round(extent / 10, 1)))

    facet_mesh = _make_facet_mesh(scene)

    pcd_cand = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(poses.positions))
    pcd_cand.paint_uniform_color(_C_CAND.tolist())

    pcd_step = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(poses.positions))
    pcd_step.paint_uniform_color(_C_OTHER.tolist())

    if len(route.order) > 1:
        ls_route = _lineset(plan_pos[route.order],
                            [[i, i + 1] for i in range(len(route.order) - 1)],
                            _C_ROUTE)
    else:
        ls_route = None

    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=r_sphere)
    sphere.compute_vertex_normals()
    sphere.paint_uniform_color(_C_CUR.tolist())

    ls_arrow   = _lineset(np.zeros((2, 3)), [[0, 1]], _C_ARROW)
    ls_frustum = _lineset(np.zeros((9, 3)), _FRUSTUM_LINES, _C_FRUSTUM)
    ls_cameras = _make_camera_pyramids(poses, selected_rows, palette, cfg)

    pcd_tracker, ls_tracker_lines = _make_tracking_geom(tracking, plan_pos)

    state = {
        "mode": "overview",
        "cur": 0,
        "pose_shown": False,
        "sphere_pos": np.zeros(3),
        "cand_shown": True,
        "cam_shown":  False,
        "trk_shown":  False,
    }

    def _set_shown(v, key: str, geoms, show: bool) -> None:
        if state[key] == show:
            return
        fn = v.add_geometry if show else v.remove_geometry
        for gm in geoms:
            if gm is not None:
                fn(gm, reset_bounding_box=False)
        state[key] = show

    def _set_layers(v, cand: bool, cameras: bool, trk: bool) -> None:
        _set_shown(v, "cand_shown", [pcd_cand], cand)
        _set_shown(v, "cam_shown", [ls_cameras], cameras)
        _set_shown(v, "trk_shown", [pcd_tracker, ls_tracker_lines], trk)

    def _show_pose(v, pose_row: int) -> None:
        pos = poses.positions[pose_row]
        sphere.translate(pos - state["sphere_pos"])
        state["sphere_pos"] = pos.copy()
        _set_points(ls_arrow, np.vstack([pos, pos + arrow_len * poses.view_dirs[pose_row]]))
        _set_points(ls_frustum, _frustum_points(pos, float(poses.yaws[pose_row]),
                                                float(poses.pitches[pose_row]), cfg))
        for gm in (sphere, ls_arrow, ls_frustum):
            if state["pose_shown"]:
                v.update_geometry(gm)
            else:
                v.add_geometry(gm, reset_bounding_box=False)
        state["pose_shown"] = True

    def _hide_pose(v) -> None:
        if state["pose_shown"]:
            for gm in (sphere, ls_arrow, ls_frustum):
                v.remove_geometry(gm, reset_bounding_box=False)
            state["pose_shown"] = False


    def _refresh_candidates(v) -> None:
        _set_layers(v, cand=True, cameras=False, trk=False)
        cur = state["cur"]
        _set_facet_colors(facet_mesh,
                          _colors_candidates(vis, coverable, cur, V_eroded))
        pcd_step.points = o3d.utility.Vector3dVector(poses.positions)
        colors_p = np.tile(_C_OTHER, (n_cand, 1))
        colors_p[cur] = _C_CUR
        pcd_step.colors = o3d.utility.Vector3dVector(colors_p)
        v.update_geometry(facet_mesh)
        v.update_geometry(pcd_step)
        _show_pose(v, cur)
        n_vis = vis.V.getrow(cur).nnz
        print(f"  Kandidat {cur + 1:>5}/{n_cand}  "
              f"({poses.positions[cur, 0]:.2f}, "
              f"{poses.positions[cur, 1]:.2f}, "
              f"{poses.positions[cur, 2]:.2f})  "
              f"Yaw {np.degrees(poses.yaws[cur]):.0f}°  "
              f"Pitch {np.degrees(poses.pitches[cur]):.0f}°  "
              f"→ {n_vis} Facetten sichtbar")

    def _refresh_plan(v) -> None:
        _set_layers(v, cand=True, cameras=False, trk=False)
        cur = state["cur"]
        _set_facet_colors(facet_mesh,
                          _colors_plan(vis, coverable, selected_rows, cur,
                                       V_eroded))
        colors_p = np.tile(_C_OTHER, (n_plan, 1))
        colors_p[cur] = _C_CUR
        pcd_step.points = o3d.utility.Vector3dVector(plan_pos)
        pcd_step.colors = o3d.utility.Vector3dVector(colors_p)
        v.update_geometry(facet_mesh)
        v.update_geometry(pcd_step)
        _show_pose(v, selected_rows[cur])
        n_vis = vis.V.getrow(selected_rows[cur]).nnz
        print(f"  Plan-Pose {cur + 1:>4}/{n_plan}  "
              f"(Pose-ID {selected_rows[cur]})  → {n_vis} Facetten sichtbar")

    def _refresh_overview(v) -> None:
        _set_layers(v, cand=True, cameras=False, trk=False)
        _set_facet_colors(facet_mesh,
                          _colors_overview(vis, coverable, selected_rows))
        pcd_step.points = o3d.utility.Vector3dVector(plan_pos)
        pcd_step.colors = o3d.utility.Vector3dVector(
            np.tile(_C_PLAN_PT, (n_plan, 1)))
        v.update_geometry(facet_mesh)
        v.update_geometry(pcd_step)
        _hide_pose(v)
        n_cov = int(
            np.asarray(vis.V[selected_rows].sum(axis=0)).ravel().astype(bool).sum())
        print(f"  Übersicht: {n_plan} Plan-Posen, "
              f"{n_cov}/{int(coverable.sum())} Facetten abgedeckt "
              f"({n_cov / max(1, int(coverable.sum())):.1%}), "
              f"Route {route.length:.1f} m")

    def _refresh_mosaic(v) -> None:
        _set_layers(v, cand=False, cameras=True, trk=True)
        _set_facet_colors(facet_mesh,
                          _colors_mosaic(vis, coverable, selected_rows, palette,
                                         overlap_ids))
        pcd_step.points = o3d.utility.Vector3dVector(plan_pos)
        pcd_step.colors = o3d.utility.Vector3dVector(palette[:n_plan])
        v.update_geometry(facet_mesh)
        v.update_geometry(pcd_step)
        _hide_pose(v)
        n_yellow = int(len(overlap_ids)) if overlap_ids is not None else 0
        n_trk = len(tracking.stations) if tracking is not None else 0
        print(f"  Mosaik: {n_plan} Posen, {n_yellow} Ueberlappungs-Facetten gelb, "
              f"{n_trk} Tracking-Standorte")


    def _step(delta: int):
        def cb(v, action, mods) -> bool:
            if action not in (1, 2):
                return False
            m = state["mode"]
            if m == "candidates":
                state["cur"] = (state["cur"] + delta) % n_cand
                _refresh_candidates(v)
            elif m == "plan":
                state["cur"] = (state["cur"] + delta) % n_plan
                _refresh_plan(v)
            return False
        return cb

    def _mode_cb(mode: str, label: str, refresh_fn):
        def cb(v, action, mods) -> bool:
            if action != 1:
                return False
            state["mode"] = mode
            state["cur"]  = 0
            print(f"\n  {label}")
            refresh_fn(v)
            return False
        return cb

    def _screenshot(v, action, mods) -> bool:
        if action != 1:
            return False
        import time as _time
        _OUT.mkdir(exist_ok=True)
        fname = (_OUT / f"inspector_{scene.name}_{state['mode']}"
                        f"_{state['cur']:04d}_{int(_time.time())}.png")
        v.capture_screen_image(str(fname), do_render=True)
        print(f"  Screenshot: {fname}")
        return False

    def _quit(v, action, mods) -> bool:
        if action == 1:
            v.close()
        return False

    viz = o3d.visualization.VisualizerWithKeyCallback()
    viz.create_window(window_name=f"vpp3d Inspector — {scene.name}",
                      width=1280, height=900)

    viz.add_geometry(facet_mesh)
    viz.add_geometry(pcd_step)
    viz.add_geometry(pcd_cand)
    viz.add_geometry(ls_ground)
    if ls_route is not None:
        viz.add_geometry(ls_route)

    ro = viz.get_render_option()
    ro.point_size   = 5.0
    ro.background_color = np.array([0.95, 0.95, 0.95])
    ro.mesh_show_back_face = True

    bindings = [
        ((_KEY_N, _KEY_RIGHT), _step(+1)),
        ((_KEY_B, _KEY_LEFT), _step(-1)),
        ((_KEY_C, _KEY_1), _mode_cb("candidates", "[C] Kandidaten-Modus", _refresh_candidates)),
        ((_KEY_P, _KEY_2), _mode_cb("plan", "[P] Plan-Modus", _refresh_plan)),
        ((_KEY_O, _KEY_3), _mode_cb("overview", "[O] Übersicht", _refresh_overview)),
        ((_KEY_M, _KEY_4), _mode_cb("mosaic", "[M] Mosaik", _refresh_mosaic)),
        ((_KEY_S,), _screenshot),
        ((_KEY_Q, _KEY_ESC), _quit),
    ]
    for keys, cb in bindings:
        for key in keys:
            viz.register_key_action_callback(key, cb)

    _refresh_overview(viz)
    viz.reset_view_point(True)

    print()
    print("  Tasten: C/1=Kandidaten  P/2=Plan  O/3=Übersicht  M/4=Mosaik")
    print("          N/→ vor  B/← zurück  S=Screenshot  Q=schließen")
    print()

    viz.run()
    viz.destroy_window()


def build_and_run(args) -> None:
    cfg = with_overrides(load_config(args.config), args)
    print(cfg.summary())
    print("=" * 64)

    scene = scene_from_args(args, cfg)
    print(scene.summary())

    raw = sample_pose_regions(scene, cfg)
    poses, d = project_and_dedup(raw, scene, cfg)
    ground_part = (f"→ Boden {d['n_after_ground']:,} "
                   if cfg.use_ground_plane else "")
    print(f"[2/3] Posen: {d['n_raw']:,} roh → Clearance {d['n_after_clearance']:,} "
          f"{ground_part}→ Dedup {d['n_after_dedup']:,}")

    vis = compute_visibility(scene, poses, cfg)
    print(vis.summary())

    erosion = cfg.erosion if args.erode is None else args.erode
    V_solve, ero = vis.V, None
    if erosion > 0:
        ero = erode_footprints(vis.V, scene.facets.centers, cfg.resolution,
                               erosion)
        V_solve = ero.V
        print("[5a] " + ero.summary())

    if args.method == "ilp":
        cover = ilp_set_cover(V_solve, k=args.k, time_limit=args.time_limit)
    elif args.method == "connected":
        cover = connected_set_cover_ilp(V_solve, cfg.min_overlap, k=args.k,
                                        time_limit=args.time_limit)
    else:
        cover = greedy_set_cover(V_solve, k=args.k)
    print(cover.summary())

    if erosion > 0 and len(cover.poses):
        extra, n_residual = cover_residual(vis.V, cover.poses)
        if n_residual:
            cover.poses = np.concatenate([cover.poses, extra])
            print(f"  Rest-Abdeckung (volles V): +{len(extra)} Posen fuer "
                  f"{n_residual} Rand-Facetten")

    selected_rows = cover.poses
    ov = None
    if not args.no_overlap:
        ov = ensure_connected(vis, cover.poses, cfg.min_overlap)
        selected_rows = ov.selected
        print(ov.summary())

    if args.refine:
        _, _, info = refine_selected_poses(
            vis, poses, scene, selected_rows, cfg,
            free=args.refine_free, gain_weight=args.gain_weight,
            max_offset=args.max_offset, sigma_scale=args.sigma_scale,
            coverage_weight=args.coverage_weight, z_min=args.z_min,
        )
        print("-" * 64)
        print("[5b] " + info.summary())
        vis = compute_visibility(scene, poses, cfg)
        if erosion > 0:
            ero = erode_footprints(vis.V, scene.facets.centers,
                                   cfg.resolution, erosion)
        n = vis.V.shape[1]
        cov = int((np.asarray(vis.V[selected_rows].sum(axis=0)).ravel() > 0).sum())
        print(f"  Abdeckung nach Verfeinerung (neu geraycastet): "
              f"{cov}/{n} ({cov / max(1, n):.1%})")

    trk = None
    if not args.no_tracking:
        trk = plan_tracking(scene, poses.positions[selected_rows], cfg)
        print(trk.summary())

    route = sequence_route(poses.positions[selected_rows], trk)
    print(route.summary())

    log = RunLog(scene.name)
    log.save_matrix_png(vis, selected_rows)

    run_inspector(scene, poses, vis, selected_rows, route, cfg,
                  overlap=ov, tracking=trk,
                  V_eroded=ero.V if ero is not None else None)


def main() -> None:
    ap = argparse.ArgumentParser(
        description="vpp3d — interaktiver 3D-Inspektor")
    add_common_args(ap)
    ap.add_argument("--method", default="greedy",
                    choices=["greedy", "ilp", "connected"])
    build_and_run(ap.parse_args())


if __name__ == "__main__":
    main()
