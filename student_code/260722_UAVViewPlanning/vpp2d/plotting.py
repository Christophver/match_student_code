from __future__ import annotations

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def _draw_scene(ax, scene):
    for poly in scene.polygons:
        p = np.vstack([poly, poly[0]])
        ax.fill(p[:, 0], p[:, 1], facecolor="0.85", edgecolor="0.4", lw=1.2, zorder=1)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")


def plot_overview(scene, poses, vis, cover, route, cfg,
                  path: str, show: bool = False):
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    fc = scene.facets

    ax = axes[0, 0]
    _draw_scene(ax, scene)
    ax.quiver(fc.centers[:, 0], fc.centers[:, 1], fc.normals[:, 0], fc.normals[:, 1],
              color="tab:blue", scale=30, width=0.003, zorder=3)
    ax.set_title(f"[1] Facetten ({len(fc)}) + Außennormalen")

    ax = axes[0, 1]
    _draw_scene(ax, scene)
    vd = poses.view_dirs
    ax.quiver(poses.positions[:, 0], poses.positions[:, 1], vd[:, 0], vd[:, 1],
              color="tab:green", scale=40, width=0.002, alpha=0.6, zorder=3)
    ax.set_title(f"[2/3] Kandidatenposen nach Projektion+Dedup ({len(poses)})")

    ax = axes[1, 0]
    _draw_scene(ax, scene)
    cov = vis.coverage_per_facet()
    sc = ax.scatter(fc.centers[:, 0], fc.centers[:, 1], c=cov, cmap="viridis",
                    s=18, zorder=3)
    fig.colorbar(sc, ax=ax, label="Posen mit Sicht (duale Sicht)")
    ax.set_title(f"[4] Coverage-Matrix: {(cov > 0).sum()}/{len(fc)} erreichbar")

    ax = axes[1, 1]
    _draw_scene(ax, scene)
    sel_V = vis.V.tocsr()[cover.poses] if len(cover.poses) else None
    covered = (np.asarray(sel_V.sum(axis=0)).ravel() > 0) if sel_V is not None \
        else np.zeros(len(fc), bool)
    ax.scatter(fc.centers[covered, 0], fc.centers[covered, 1], c="tab:green",
               s=12, zorder=2, label="abgedeckt")
    ax.scatter(fc.centers[~covered, 0], fc.centers[~covered, 1], c="tab:red",
               s=12, zorder=2, label="offen")

    pp = poses.positions[cover.poses]
    pvd = poses.view_dirs[cover.poses]
    ax.quiver(pp[:, 0], pp[:, 1], pvd[:, 0], pvd[:, 1], color="black",
              scale=22, width=0.005, zorder=6)

    if len(route.order):
        rp = pp[route.order]
        ax.plot(rp[:, 0], rp[:, 1], "-", color="tab:orange", lw=1.0, alpha=0.8,
                zorder=4)

    ax.set_title(f"[6/7] {len(cover.poses)} Posen, Weg {route.length:.0f} m")
    ax.legend(loc="upper right", fontsize=8)

    fig.suptitle(f"vpp2d — {scene.name} — {cover.method}", fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    fig.savefig(path, dpi=110)
    if show:
        import matplotlib.pyplot as _plt
        _plt.show()
    plt.close(fig)
    return path
