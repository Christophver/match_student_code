from __future__ import annotations

import numpy as np

C_COVERED = (0.19, 0.64, 0.33)
C_OPEN = (0.89, 0.29, 0.20)
C_UNREACH = (0.55, 0.55, 0.55)
C_OVERLAP = (1.0, 0.70, 0.0)
C_STATION = (0.10, 0.45, 0.90)


def _selected_ids(cover, selected):
    return cover.poses if selected is None else np.asarray(selected, np.int64)


def _facet_status(scene, vis, cover, selected=None):
    n = len(scene.facets)
    coverable = vis.coverage_per_facet() > 0
    sel_rows = _selected_ids(cover, selected)
    if len(sel_rows):
        covered = np.asarray(vis.V.tocsr()[sel_rows].sum(axis=0)).ravel() > 0
    else:
        covered = np.zeros(n, dtype=bool)
    return covered, coverable & ~covered, ~coverable


def plot_overview_png(scene, poses, vis, cover, route, cfg, path: str,
                      show: bool = False, selected=None,
                      overlap=None, tracking=None) -> str:
    import matplotlib
    matplotlib.use("TkAgg" if show else "Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fc = scene.facets
    sel_rows = _selected_ids(cover, selected)
    covered, open_, unreach = _facet_status(scene, vis, cover, selected)

    overlap_mask = np.zeros(len(fc), dtype=bool)
    if overlap is not None:
        overlap_mask[overlap.overlap_facets] = True
        covered = covered & ~overlap_mask

    fig = plt.figure(figsize=(15, 7))

    ax1 = fig.add_subplot(1, 2, 1, projection="3d")
    _draw_mesh(ax1, scene, Poly3DCollection)
    pp = poses.positions
    ax1.scatter(pp[:, 0], pp[:, 1], pp[:, 2], c="tab:green", s=4, alpha=0.4)
    ax1.set_title(f"[2/3] Kandidatenposen nach Projektion+Dedup ({len(poses)})")
    _equal_3d(ax1, scene)

    ax2 = fig.add_subplot(1, 2, 2, projection="3d")
    _draw_mesh(ax2, scene, Poly3DCollection)
    for mask, col, lbl in ((covered, C_COVERED, "abgedeckt"),
                           (overlap_mask, C_OVERLAP, "Registrierungs-Ueberlappung"),
                           (open_, C_OPEN, "offen"),
                           (unreach, C_UNREACH, "unerreichbar")):
        if mask.any():
            ax2.scatter(fc.centers[mask, 0], fc.centers[mask, 1], fc.centers[mask, 2],
                        c=[col], s=8, label=lbl)
    sp = poses.positions[sel_rows]
    sd = poses.view_dirs[sel_rows]
    if len(sp):
        ax2.scatter(sp[:, 0], sp[:, 1], sp[:, 2], c="black", s=30, marker="o")
        ax2.quiver(sp[:, 0], sp[:, 1], sp[:, 2], sd[:, 0], sd[:, 1], sd[:, 2],
                   length=0.6 * cfg.d_opt, color="black", linewidth=1.0,
                   normalize=True)
    if len(route.order):
        rp = sp[route.order]
        ax2.plot(rp[:, 0], rp[:, 1], rp[:, 2], "-", color="tab:orange", lw=1.0)

    if tracking is not None and len(tracking.stations):
        st = tracking.stations
        ax2.scatter(st[:, 0], st[:, 1], st[:, 2], c=[C_STATION], s=70,
                    marker="^", edgecolor="black", label="Tracking-Standort")
        for k, a in enumerate(tracking.assignment):
            if a < 0:
                continue
            o, t = sp[k], st[a]
            ax2.plot([o[0], t[0]], [o[1], t[1]], [o[2], t[2]],
                     "-", color=C_STATION, lw=0.4, alpha=0.5)

    n_add = 0 if overlap is None else len(overlap.added)
    ax2.set_title(f"[5/6/7] {cover.method}: {len(sel_rows)} Posen "
                  f"(+{n_add} Bruecken), Weg {route.length:.0f} m")
    ax2.legend(loc="upper right", fontsize=8)
    _equal_3d(ax2, scene)

    fig.suptitle(f"vpp3d — {scene.name} — {cover.method}", fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(path, dpi=120)
    if show:
        plt.show()
    plt.close(fig)
    return path


def _draw_mesh(ax, scene, Poly3DCollection) -> None:
    coll = Poly3DCollection(scene.vertices[scene.triangles], alpha=0.12,
                            facecolor="0.6", edgecolor="0.4", linewidths=0.2)
    ax.add_collection3d(coll)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")


def _equal_3d(ax, scene) -> None:
    lo, hi = scene.bounds
    c = 0.5 * (lo + hi)
    r = 0.5 * float((hi - lo).max()) + 1.0
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)
    try:
        ax.set_box_aspect((1, 1, 1))
    except Exception:
        pass


def show_open3d(scene, poses, vis, cover, route, cfg, selected=None,
                overlap=None, tracking=None) -> None:
    import open3d as o3d

    def lineset(pts, idx, color):
        ls = o3d.geometry.LineSet(
            o3d.utility.Vector3dVector(np.asarray(pts, dtype=float)),
            o3d.utility.Vector2iVector(np.asarray(idx)))
        ls.paint_uniform_color(list(color))
        return ls

    geoms = []
    mesh = o3d.geometry.TriangleMesh(scene.mesh)
    mesh.paint_uniform_color([0.8, 0.8, 0.82])
    mesh.compute_vertex_normals()
    geoms.append(mesh)

    sel_rows = _selected_ids(cover, selected)
    covered, open_, unreach = _facet_status(scene, vis, cover, selected)
    fc = scene.facets
    colors = np.empty((len(fc), 3))
    colors[covered] = C_COVERED
    colors[open_] = C_OPEN
    colors[unreach] = C_UNREACH
    if overlap is not None:
        colors[overlap.overlap_facets] = C_OVERLAP
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(fc.centers))
    pc.colors = o3d.utility.Vector3dVector(colors)
    geoms.append(pc)

    sp = poses.positions[sel_rows]
    sd = poses.view_dirs[sel_rows]
    r = max(0.05, 0.04 * cfg.d_opt)
    line_pts, line_idx = [], []
    for k, (p, d) in enumerate(zip(sp, sd)):
        ball = o3d.geometry.TriangleMesh.create_sphere(radius=r)
        ball.translate(p)
        ball.paint_uniform_color([0.0, 0.0, 0.0])
        ball.compute_vertex_normals()
        geoms.append(ball)
        line_pts += [p, p + d * 0.6 * cfg.d_opt]
        line_idx.append([2 * k, 2 * k + 1])
    if line_pts:
        geoms.append(lineset(line_pts, line_idx, (0.1, 0.1, 0.1)))

    if len(route.order) > 1:
        rp = sp[route.order]
        geoms.append(lineset(rp, [[i, i + 1] for i in range(len(rp) - 1)],
                             (1.0, 0.55, 0.0)))

    if tracking is not None and len(tracking.stations):
        st = tracking.stations
        box = max(0.1, 0.06 * cfg.d_opt)
        for a_st in st:
            cube = o3d.geometry.TriangleMesh.create_box(box, box, box)
            cube.translate(a_st - box / 2.0)
            cube.paint_uniform_color(list(C_STATION))
            cube.compute_vertex_normals()
            geoms.append(cube)
        los_pts, los_idx = [], []
        for k, a in enumerate(tracking.assignment):
            if a < 0:
                continue
            los_pts += [sp[k], st[a]]
            los_idx.append([len(los_pts) - 2, len(los_pts) - 1])
        if los_pts:
            geoms.append(lineset(los_pts, los_idx, C_STATION))

    o3d.visualization.draw_geometries(
        geoms, window_name=f"vpp3d — {scene.name} — {cover.method}")
