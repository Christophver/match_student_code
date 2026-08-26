from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    sys.stdout.reconfigure(encoding="utf-8")
except (AttributeError, ValueError):
    pass

import numpy as np

from .config import load_config, with_overrides
from . import scene as scene_mod
from .scene import scene_from_args
from .candidates import sample_pose_regions, project_and_dedup
from .visibility import compute_visibility
from .setcover import greedy_set_cover, ilp_set_cover, connected_set_cover_ilp
from .erosion import erode_footprints, cover_residual
from .overlap import ensure_connected
from .refine import refine_selected_poses
from .tracking import plan_tracking
from .sequencing import sequence_route
from .viz import plot_overview_png, show_open3d
from .runlog import RunLog

_OUT = Path(__file__).with_name("output")


def add_common_args(ap: argparse.ArgumentParser) -> None:
    ap.add_argument("--scene", default="box", choices=list(scene_mod.SCENES),
                    help="synthetische Testszene")
    ap.add_argument("--mesh", default=None,
                    help="statt Szene: reales Mesh/Punktwolke (STL/PLY)")
    ap.add_argument("--assume-convex", action="store_true",
                    help="Normalen radial nach aussen orientieren (Punktwolken)")
    ap.add_argument("--resolution", type=float, default=None,
                    help="Facetten-Aufloesung [m] ueberschreiben (groesser = "
                         "weniger Facetten; noetig fuer grosse Objekte wie EasyCube)")
    ap.add_argument("--k", type=int, default=1,
                    help="k-Coverage: jede Facette von mind. k Posen sehen lassen "
                         "(Redundanz fuer die Registrierung; Default 1 = "
                         "reine Vollabdeckung). Bedarf wird auf die erreichbare "
                         "Abdeckung gekappt.")
    ap.add_argument("--baseline", type=float, default=None,
                    help="Basislinie ueberschreiben (0 = Einzelsicht-Ablation)")
    ap.add_argument("--time-limit", type=float, default=60.0, help="Zeitlimit ILP [s]")
    ap.add_argument("--erode", type=float, default=None,
                    help="[5a] Footprint-Erosion [m]: Overlap per Konstruktion "
                         "— Kandidaten-Footprints vor dem Set Cover um diesen "
                         "Rand erodieren (benachbarte Scans ueberlappen um "
                         "~2*erode). Ueberschreibt registration.erosion aus "
                         "der config.toml; 0 = aus.")
    ap.add_argument("--no-overlap", action="store_true",
                    help="Konnektivitaets-Reparatur des Registrierungsgraphen "
                         "ueberspringen")
    ap.add_argument("--no-tracking", action="store_true",
                    help="LoS-Tracking-Standorte ueberspringen")
    ap.add_argument("--refine", action="store_true",
                    help="[5b] gewaehlte Posen lokal auf ein Qualitaetsoptimum "
                         "verschieben (glatte Score, abdeckungsbewusst)")
    ap.add_argument("--refine-free", action="store_true",
                    help="ABLATION zu --refine: ohne Facetten-Zuweisung optimieren "
                         "(Posen kollabieren, Abdeckung bricht ein)")
    ap.add_argument("--gain-weight", type=float, default=0.0,
                    help="--refine: > 0 nimmt benachbarte Facetten mit Gewicht g in "
                         "die Score auf (Abdeckungsmaximierung; d_max-Drift durch "
                         "die Distanz-Glocke gebremst; Default 0 = aus)")
    ap.add_argument("--max-offset", type=float, default=None,
                    help="--refine: max. Suchradius je Pose [m] (Default 0.5·d_opt)")
    ap.add_argument("--sigma-scale", type=float, default=1.5,
                    help="--refine: Breite der Distanz-Glocke (groesser = toleranter)")
    ap.add_argument("--coverage-weight", type=float, default=4.0,
                    help="--refine: Abdeckungs-Barriere je zugewiesener Facette "
                         "(0 = aus)")
    ap.add_argument("--z-min", type=float, default=None,
                    help="--refine: Bodenfilter, Posen nicht unter z_min schieben")
    ap.add_argument("--no-ground-plane", action="store_true",
                    help="Bodenebene-Filter deaktivieren (Posen duerfen unter "
                         "z_boden + safety_distance liegen)")
    ap.add_argument("--config", default=None, help="alternative config.toml")


def _apply_refinement(vis, poses, scene, selected, cfg, args):
    before, refined, info = refine_selected_poses(
        vis, poses, scene, selected, cfg,
        free=args.refine_free, gain_weight=args.gain_weight,
        max_offset=args.max_offset, sigma_scale=args.sigma_scale,
        coverage_weight=args.coverage_weight, z_min=args.z_min,
    )
    print("-" * 64)
    print("[5b] " + info.summary())

    vis_before = compute_visibility(scene, before, cfg)
    vis_after = compute_visibility(scene, refined, cfg)
    cov_b = int((np.asarray(vis_before.V.sum(axis=0)).ravel() > 0).sum())
    cov_a = int((np.asarray(vis_after.V.sum(axis=0)).ravel() > 0).sum())
    n = vis.V.shape[1]
    print(f"  Abdeckung (Teilmenge, neu geraycastet): {cov_b}/{n} ({cov_b / max(1, n):.1%})"
          f" -> {cov_a}/{n} ({cov_a / max(1, n):.1%})")
    return poses


def run(args) -> None:
    cfg = with_overrides(load_config(args.config), args)
    print(cfg.summary())
    print("=" * 64)

    scene = scene_from_args(args, cfg)
    print(scene.summary())

    raw = sample_pose_regions(scene, cfg)
    poses, d = project_and_dedup(raw, scene, cfg)
    ground_part = (f"-> Boden {d['n_after_ground']:,} "
                   if cfg.use_ground_plane else "")
    print(f"[2/3] Posen: roh {d['n_raw']:,} -> Clearance {d['n_after_clearance']:,} "
          f"{ground_part}"
          f"-> Machbarkeit {d['n_after_feasibility']:,} -> Dedup {d['n_after_dedup']:,} "
          f"(Faktor {d['n_after_feasibility'] / max(1, d['n_after_dedup']):.1f})")

    vis = compute_visibility(scene, poses, cfg)
    print(vis.summary())
    if cfg.baseline > 0:
        s = vis.stats
        lost = s.get("occlusion_proj", 0) - s.get("dual", 0)
        print(f"      duale Sicht verwirft {lost:,} Paare, die die Einzelsicht "
              f"zulassen wuerde (Kamera verdeckt).")

    erosion = cfg.erosion if args.erode is None else args.erode
    V_solve = vis.V
    if erosion > 0:
        ero = erode_footprints(vis.V, scene.facets.centers, cfg.resolution,
                               erosion)
        V_solve = ero.V
        print("-" * 64)
        print("[5a] " + ero.summary())

    method_map = {"both": ["greedy", "ilp"],
                  "all": ["greedy", "ilp", "connected"]}
    methods = method_map.get(args.method, [args.method])

    def _solve(meth):
        if meth == "greedy":
            return greedy_set_cover(V_solve, k=args.k)
        if meth == "ilp":
            return ilp_set_cover(V_solve, k=args.k, time_limit=args.time_limit)
        return connected_set_cover_ilp(V_solve, cfg.min_overlap, k=args.k,
                                       time_limit=args.time_limit)

    results = {}
    for meth in methods:
        res = _solve(meth)
        results[meth] = res
        print("-" * 64)
        print(res.summary())
        if erosion > 0 and len(res.poses):
            extra, n_residual = cover_residual(vis.V, res.poses)
            if n_residual:
                res.poses = np.concatenate([res.poses, extra])
                print(f"  Rest-Abdeckung (volles V)       : +{len(extra)} "
                      f"Posen fuer {n_residual} Rand-Facetten")
            cov = np.asarray(vis.V[res.poses].sum(axis=0)).ravel() > 0
            reach = np.asarray(vis.V.sum(axis=0)).ravel() > 0
            n_real = int((cov & reach).sum())
            print(f"  reale Abdeckung (volles V)      : "
                  f"{n_real}/{int(reach.sum())} "
                  f"({n_real / max(1, int(reach.sum())):.1%})")

    if "greedy" in results and "ilp" in results:
        g, il = results["greedy"], results["ilp"]
        print("-" * 64)
        print(f"Greedy vs. ILP: Posen {len(g.poses)} vs. {len(il.poses)} "
              f"(Faktor {len(g.poses) / max(1, len(il.poses)):.2f})")

    _OUT.mkdir(exist_ok=True)
    log = RunLog(scene.name)
    for meth, res in results.items():
        selected = res.poses
        ov = None
        if not args.no_overlap:
            ov = ensure_connected(vis, res.poses, cfg.min_overlap)
            selected = ov.selected
            print("-" * 64)
            print(ov.summary())

        if args.refine:
            poses = _apply_refinement(vis, poses, scene, selected, cfg, args)

        trk = None
        if not args.no_tracking:
            trk = plan_tracking(scene, poses.positions[selected], cfg)
            print("-" * 64)
            print(trk.summary())

        route = sequence_route(poses.positions[selected], trk)
        print(f"[7] {meth}: {route.summary()}")
        png = _OUT / f"{scene.name}_{meth}.png"
        plot_overview_png(scene, poses, vis, res, route, cfg, str(png),
                          show=False, selected=selected, overlap=ov, tracking=trk)
        print(f"    -> {png}")
        log.save_matrix_png(vis, selected)
        log.copy_png(png)
        if args.show:
            show_open3d(scene, poses, vis, res, route, cfg,
                        selected=selected, overlap=ov, tracking=trk)


def main() -> None:
    ap = argparse.ArgumentParser(description="vpp3d — 3D-Prototyp (alternativer VPP-Ansatz)")
    add_common_args(ap)
    ap.add_argument("--method", default="greedy",
                    choices=["greedy", "ilp", "connected", "both", "all"],
                    help="greedy/ilp: Set Cover + Konnektivitaets-Reparatur; "
                         "connected: gemeinsames Connected-Set-Cover-ILP "
                         "(Abdeckung + Zusammenhang exakt); both=greedy+ilp, "
                         "all=greedy+ilp+connected")
    ap.add_argument("--show", action="store_true", help="Open3D-Ansicht der Loesung")
    run(ap.parse_args())


if __name__ == "__main__":
    main()
