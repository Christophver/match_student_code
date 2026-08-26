from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    sys.stdout.reconfigure(encoding="utf-8")
except (AttributeError, ValueError):
    pass

from .config import load_config, with_overrides
from . import scene as scene_mod
from .scene import scene_from_args
from .candidates import sample_pose_regions, project_and_dedup
from .visibility import compute_visibility
from .setcover import greedy_set_cover
from .viz import _draw_mesh, _equal_3d

_OUT = Path(__file__).with_name("output")

C_NONCOVER = "0.55"
C_OPEN = "#e34a33"
C_DONE = "#31a354"
C_NEW = "#fd8d3c"
C_OVERLAP = "#3182bd"


class StepData:
    def __init__(self, scene, poses, vis, cfg, trace):
        self.scene = scene
        self.poses = poses
        self.vis = vis
        self.cfg = cfg
        self.trace = trace
        self.coverable = vis.coverage_per_facet() > 0
        self.n_coverable = int(self.coverable.sum())
        self.n_steps = len(trace)


def build(args) -> StepData:
    cfg = with_overrides(load_config(args.config), args)
    scene = scene_from_args(args, cfg)
    print(scene.summary())

    raw = sample_pose_regions(scene, cfg)
    poses, _ = project_and_dedup(raw, scene, cfg)
    vis = compute_visibility(scene, poses, cfg)

    trace: list = []
    res = greedy_set_cover(vis.V, trace=trace)
    print(res.summary())
    print(f"  -> {len(trace)} Greedy-Schritte aufgezeichnet.")
    return StepData(scene, poses, vis, cfg, trace)


def draw_step(ax, data: StepData, k: int) -> None:
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    elev, azim = ax.elev, ax.azim
    ax.clear()
    sc = data.scene
    fc = sc.facets
    _draw_mesh(ax, sc, Poly3DCollection)

    if k == 0:
        covered_before = np.zeros(len(fc), dtype=bool)
        new = np.empty(0, dtype=int)
        overlap = np.empty(0, dtype=int)
        pose = None
    else:
        st = data.trace[k - 1]
        covered_before = st["covered_before"]
        new = st["new_facets"]
        overlap = st["overlap_facets"]
        pose = st["pose"]

    open_mask = data.coverable & ~covered_before
    if k > 0:
        open_mask[new] = False
    unreach = ~data.coverable
    if unreach.any():
        ax.scatter(fc.centers[unreach, 0], fc.centers[unreach, 1], fc.centers[unreach, 2],
                   c=C_NONCOVER, s=8, label="unerreichbar")
    if covered_before.any():
        ax.scatter(fc.centers[covered_before, 0], fc.centers[covered_before, 1],
                   fc.centers[covered_before, 2], c=C_DONE, s=10, label="abgedeckt")
    if open_mask.any():
        ax.scatter(fc.centers[open_mask, 0], fc.centers[open_mask, 1],
                   fc.centers[open_mask, 2], c=C_OPEN, s=10, label="offen")

    if pose is not None:
        p = data.poses.positions[pose]
        d = data.poses.view_dirs[pose]
        for f, col, lw, al in ([(f, C_OVERLAP, 0.5, 0.6) for f in overlap]
                               + [(f, C_NEW, 0.7, 0.8) for f in new]):
            c = fc.centers[f]
            ax.plot([p[0], c[0]], [p[1], c[1]], [p[2], c[2]],
                    color=col, lw=lw, alpha=al)
        if len(overlap):
            ax.scatter(fc.centers[overlap, 0], fc.centers[overlap, 1], fc.centers[overlap, 2],
                       c=C_OVERLAP, s=40, edgecolor="white", lw=0.4, label="Ueberlappung")
        if len(new):
            ax.scatter(fc.centers[new, 0], fc.centers[new, 1], fc.centers[new, 2],
                       c=C_NEW, s=40, edgecolor="white", lw=0.4, label="neu erfasst")
        ax.scatter([p[0]], [p[1]], [p[2]], c="black", s=45, marker="o")
        ax.quiver(p[0], p[1], p[2], d[0], d[1], d[2],
                  length=0.6 * data.cfg.d_opt, color="black", linewidth=1.5,
                  normalize=True)

    done = int(covered_before.sum()) + (len(new) if k > 0 else 0)
    frac = done / max(1, data.n_coverable)
    if k == 0:
        title = (f"Schritt 0 / {data.n_steps} — Start: 0 Posen, "
                 f"0 / {data.n_coverable} erfasst (0 %)")
    else:
        title = (f"Schritt {k} / {data.n_steps} — Pose #{pose}\n"
                 f"+{len(new)} neu, {len(overlap)} Ueberlappung   |   "
                 f"{done} / {data.n_coverable} erfasst ({frac:.0%}), {k} Posen")
    ax.set_title(title, fontsize=10)
    ax.legend(loc="upper right", fontsize=7, framealpha=0.9)
    _equal_3d(ax, sc)
    ax.view_init(elev=elev, azim=azim)


def save_frames(data: StepData, outdir: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    outdir.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(10, 9))
    ax = fig.add_subplot(111, projection="3d")
    ax.view_init(elev=22, azim=-60)
    for k in range(data.n_steps + 1):
        draw_step(ax, data, k)
        fig.tight_layout()
        fig.savefig(outdir / f"step_{k:03d}.png", dpi=110)
    plt.close(fig)
    print(f"  -> {data.n_steps + 1} Frames in {outdir}")


def interactive(data: StepData) -> None:
    import matplotlib
    try:
        matplotlib.use("TkAgg")
    except Exception:
        pass
    import matplotlib.pyplot as plt

    state = {"k": 0}
    fig = plt.figure(figsize=(10, 9))
    ax = fig.add_subplot(111, projection="3d")
    ax.view_init(elev=22, azim=-60)

    def redraw():
        draw_step(ax, data, state["k"])
        fig.canvas.draw_idle()

    def on_key(event):
        if event.key in ("right", "n", " "):
            state["k"] = min(state["k"] + 1, data.n_steps)
            redraw()
        elif event.key in ("left", "b"):
            state["k"] = max(state["k"] - 1, 0)
            redraw()
        elif event.key == "home":
            state["k"] = 0; redraw()
        elif event.key == "end":
            state["k"] = data.n_steps; redraw()
        elif event.key == "s":
            _OUT.mkdir(exist_ok=True)
            f = _OUT / f"{data.scene.name}_step_{state['k']:03d}.png"
            fig.savefig(f, dpi=120); print(f"  gespeichert: {f}")
        elif event.key == "q":
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)
    print("Navigation: → / n = vor, ← / b = zurueck, Home/End, s = speichern, q = schliessen")
    redraw()
    plt.show()


def main() -> None:
    ap = argparse.ArgumentParser(description="vpp3d — Schritt-fuer-Schritt-Debug-Viewer")
    ap.add_argument("--scene", default="notched_box", choices=list(scene_mod.SCENES))
    ap.add_argument("--mesh", default=None,
                    help="statt Szene: reales Mesh/Punktwolke (STL/PLY)")
    ap.add_argument("--assume-convex", action="store_true",
                    help="Normalen radial nach aussen orientieren (Punktwolken)")
    ap.add_argument("--resolution", type=float, default=None,
                    help="Facetten-Aufloesung [m] ueberschreiben")
    ap.add_argument("--baseline", type=float, default=None)
    ap.add_argument("--config", default=None)
    ap.add_argument("--show", action="store_true",
                    help="interaktiver Navigator statt PNG-Frames")
    args = ap.parse_args()

    data = build(args)
    if args.show:
        interactive(data)
    else:
        save_frames(data, _OUT / f"steps_{data.scene.name}_greedy")


if __name__ == "__main__":
    main()
