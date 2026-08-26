from __future__ import annotations

import argparse
import dataclasses
import sys
from pathlib import Path

try:
    sys.stdout.reconfigure(encoding="utf-8")
except (AttributeError, ValueError):
    pass

from .config import load_config
from . import scene as scene_mod
from .candidates import sample_pose_regions, project_and_dedup
from .visibility import compute_visibility
from .setcover import greedy_set_cover, ilp_set_cover
from .sequencing import sequence_route
from .plotting import plot_overview

_OUT = Path(__file__).with_name("output")


def build_scene_from_args(args, cfg):
    if args.stl:
        return scene_mod.from_stl_slice(
            args.stl, level=args.stl_level, resolution=cfg.resolution)
    return scene_mod.SCENES[args.scene](cfg.resolution)


def run(args) -> None:
    cfg = load_config(args.config)
    if args.baseline is not None:
        cfg = dataclasses.replace(cfg, baseline=args.baseline)

    print(cfg.summary())
    print("=" * 64)

    scene = build_scene_from_args(args, cfg)
    print(scene.summary())

    raw = sample_pose_regions(scene, cfg)
    poses, dstats = project_and_dedup(raw, scene, cfg)
    print(f"[2/3] Posen: roh {dstats['n_raw']:,} -> Clearance "
          f"{dstats['n_after_clearance']:,} -> Dedup {dstats['n_after_dedup']:,} "
          f"(Faktor {dstats['n_after_clearance'] / max(1, dstats['n_after_dedup']):.1f})")

    vis = compute_visibility(scene, poses, cfg)
    print(vis.summary())
    if cfg.baseline > 0:
        s = vis.stats
        lost = s["occlusion_single"] - s["dual"]
        print(f"      duale Sicht verwirft {lost:,} Paare, die die Einzelsicht "
              f"zulassen würde (Kamera verdeckt).")

    methods = ["greedy", "ilp"] if args.method == "both" else [args.method]
    results = {}
    for meth in methods:
        if meth == "greedy":
            res = greedy_set_cover(vis.V)
        else:
            res = ilp_set_cover(vis.V, time_limit=args.time_limit)
        results[meth] = res
        print("-" * 64)
        print(res.summary())

    if "greedy" in results and "ilp" in results:
        g, il = results["greedy"], results["ilp"]
        print("-" * 64)
        print(f"Greedy vs. ILP: Posen {len(g.poses)} vs. {len(il.poses)} "
              f"(Faktor {len(g.poses) / max(1, len(il.poses)):.2f})")

    _OUT.mkdir(exist_ok=True)
    for meth, res in results.items():
        route = sequence_route(poses.positions[res.poses])
        print(f"[7] {meth}: {route.summary()}")
        png = _OUT / f"{scene.name}_{meth}.png"
        plot_overview(scene, poses, vis, res, route, cfg, str(png), show=args.show)
        print(f"    -> {png}")


def main() -> None:
    ap = argparse.ArgumentParser(description="vpp2d — 2D-Prototyp (alternativer VPP-Ansatz)")
    ap.add_argument("--scene", default="box", choices=list(scene_mod.SCENES),
                    help="synthetische Testszene")
    ap.add_argument("--stl", default=None,
                    help="statt Szene: 2D-Schnitt durch ein STL-Mesh")
    ap.add_argument("--stl-level", type=float, default=None,
                    help="Schnitthöhe z für --stl (Default: Mitte)")
    ap.add_argument("--method", default="greedy", choices=["greedy", "ilp", "both"])
    ap.add_argument("--baseline", type=float, default=None,
                    help="Basislinie überschreiben (0 = Einzelsicht-Ablation)")
    ap.add_argument("--time-limit", type=float, default=60.0,
                    help="Zeitlimit ILP [s]")
    ap.add_argument("--config", default=None, help="alternative config.toml")
    ap.add_argument("--show", action="store_true", help="Plots interaktiv anzeigen")
    args = ap.parse_args()

    if args.show:
        import matplotlib
        matplotlib.use("TkAgg")
    run(args)


if __name__ == "__main__":
    main()
