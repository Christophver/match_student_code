from __future__ import annotations

import argparse
import dataclasses
import sys
from pathlib import Path

import numpy as np

try:
    sys.stdout.reconfigure(encoding="utf-8")
except (AttributeError, ValueError):
    pass

from .config import load_config
from . import scene as scene_mod
from .candidates import sample_pose_regions, project_and_dedup
from .visibility import compute_visibility
from .setcover import greedy_set_cover
from .plotting import _draw_scene

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
    cfg = load_config(args.config)
    if args.baseline is not None:
        cfg = dataclasses.replace(cfg, baseline=args.baseline)

    if args.stl:
        scene = scene_mod.from_stl_slice(args.stl, level=args.stl_level,
                                         resolution=cfg.resolution)
    else:
        scene = scene_mod.SCENES[args.scene](cfg.resolution)

    raw = sample_pose_regions(scene, cfg)
    poses, _ = project_and_dedup(raw, scene, cfg)
    vis = compute_visibility(scene, poses, cfg)

    trace: list = []
    res = greedy_set_cover(vis.V, trace=trace)
    print(f"{scene.name}: {res.summary()}")
    print(f"  -> {len(trace)} Greedy-Schritte aufgezeichnet.")
    return StepData(scene, poses, vis, cfg, trace)


def draw_step(ax, data: StepData, k: int) -> None:
    from matplotlib.patches import Wedge

    ax.clear()
    sc = data.scene
    fc = sc.facets
    cfg = data.cfg
    _draw_scene(ax, sc)

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
    ax.scatter(fc.centers[~data.coverable, 0], fc.centers[~data.coverable, 1],
               c=C_NONCOVER, s=12, zorder=2, label="unerreichbar")
    ax.scatter(fc.centers[covered_before, 0], fc.centers[covered_before, 1],
               c=C_DONE, s=14, zorder=2, label="abgedeckt")
    ax.scatter(fc.centers[open_mask, 0], fc.centers[open_mask, 1],
               c=C_OPEN, s=14, zorder=2, label="offen")

    if pose is not None:
        p = data.poses.positions[pose]
        yaw = data.poses.yaws[pose]
        yaw_deg = np.degrees(yaw)
        half = np.degrees(cfg.fov_rad / 2.0)
        ax.add_patch(Wedge(p, cfg.d_max, yaw_deg - half, yaw_deg + half,
                          width=cfg.d_max - cfg.d_min, facecolor="gold",
                          alpha=0.18, edgecolor="goldenrod", lw=0.8, zorder=3))
        for f in overlap:
            ax.plot([p[0], fc.centers[f, 0]], [p[1], fc.centers[f, 1]],
                    color=C_OVERLAP, lw=0.6, alpha=0.7, zorder=4)
        for f in new:
            ax.plot([p[0], fc.centers[f, 0]], [p[1], fc.centers[f, 1]],
                    color=C_NEW, lw=0.8, alpha=0.9, zorder=4)
        if len(overlap):
            ax.scatter(fc.centers[overlap, 0], fc.centers[overlap, 1],
                       c=C_OVERLAP, s=42, edgecolor="white", lw=0.5, zorder=7,
                       label="Überlappung")
        if len(new):
            ax.scatter(fc.centers[new, 0], fc.centers[new, 1],
                       c=C_NEW, s=42, edgecolor="white", lw=0.5, zorder=7,
                       label="neu erfasst")
        u = np.array([np.cos(yaw), np.sin(yaw)])
        ax.plot(*p, marker="o", color="black", ms=8, zorder=8)
        ax.annotate("", xy=p + u * 0.9, xytext=p,
                    arrowprops=dict(arrowstyle="-|>", color="black", lw=1.5),
                    zorder=8)

    done = int(covered_before.sum()) + (len(new) if k > 0 else 0)
    frac = done / max(1, data.n_coverable)
    if k == 0:
        title = (f"Schritt 0 / {data.n_steps} — Start: 0 Posen, "
                 f"0 / {data.n_coverable} erfasst (0 %)")
    else:
        title = (f"Schritt {k} / {data.n_steps} — Pose #{pose}\n"
                 f"+{len(new)} neu, {len(overlap)} Überlappung   |   "
                 f"{done} / {data.n_coverable} erfasst ({frac:.0%}), {k} Posen")
    ax.set_title(title, fontsize=10)
    ax.legend(loc="upper right", fontsize=7, framealpha=0.9)


def save_frames(data: StepData, outdir: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    outdir.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(9, 8))
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
    fig, ax = plt.subplots(figsize=(9, 8))

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
    print("Navigation: → / n = vor, ← / b = zurück, Home/End, s = speichern, q = schließen")
    redraw()
    plt.show()


def main() -> None:
    ap = argparse.ArgumentParser(description="vpp2d — Schritt-für-Schritt-Debug-Viewer")
    ap.add_argument("--scene", default="notched_box", choices=list(scene_mod.SCENES))
    ap.add_argument("--stl", default=None)
    ap.add_argument("--stl-level", type=float, default=None)
    ap.add_argument("--baseline", type=float, default=None)
    ap.add_argument("--config", default=None)
    ap.add_argument("--show", action="store_true",
                    help="interaktiver Navigator statt PNG-Frames")
    args = ap.parse_args()

    data = build(args)
    if args.show:
        interactive(data)
    else:
        name = data.scene.name
        save_frames(data, _OUT / f"steps_{name}_greedy")


if __name__ == "__main__":
    main()
