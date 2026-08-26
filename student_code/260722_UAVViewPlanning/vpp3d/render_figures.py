from __future__ import annotations

import argparse
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
from .setcover import ilp_set_cover, greedy_set_cover
from .overlap import ensure_connected
from .tracking import plan_tracking
from .sequencing import sequence_route
from . import inspector as insp

_FIG_DIR = Path(__file__).resolve().parents[2] / "LaTex" / "abbildung"
_SCRATCH = Path(__file__).with_name("output") / "figtest"

TEXTWIDTH_PT = 469.47
BODY_PT = 11.0
_FONT_PATH = r"C:\Windows\Fonts\arial.ttf"

_WHITE = np.array([255, 255, 255], dtype=np.uint8)


def build_solution(scene, cfg, method: str = "ilp", time_limit: float = 60.0):
    raw = sample_pose_regions(scene, cfg)
    poses, _ = project_and_dedup(raw, scene, cfg)
    vis = compute_visibility(scene, poses, cfg)

    if method == "ilp":
        cover = ilp_set_cover(vis.V, k=1, time_limit=time_limit)
    else:
        cover = greedy_set_cover(vis.V, k=1)

    ov = ensure_connected(vis, cover.poses, cfg.min_overlap)
    selected = ov.selected
    trk = plan_tracking(scene, poses.positions[selected], cfg)
    route = sequence_route(poses.positions[selected], trk)
    return poses, vis, selected, route, ov, trk


def mosaic_geometries(scene, poses, vis, selected, route, ov, trk, cfg):
    import open3d as o3d

    coverable = vis.coverage_per_facet() > 0
    palette = insp._distinct_colors(len(selected))
    plan_pos = poses.positions[selected]
    overlap_ids = ov.overlap_facets if ov is not None else None

    geoms = []

    facet_mesh = insp._make_facet_mesh(scene)
    insp._set_facet_colors(
        facet_mesh,
        insp._colors_mosaic(vis, coverable, selected, palette, overlap_ids))
    geoms.append(facet_mesh)

    pcd_poses = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(plan_pos))
    pcd_poses.colors = o3d.utility.Vector3dVector(palette[:len(selected)])
    geoms.append(pcd_poses)

    geoms.append(insp._make_camera_pyramids(poses, selected, palette, cfg))

    if len(route.order) > 1:
        geoms.append(insp._lineset(
            plan_pos[route.order],
            [[i, i + 1] for i in range(len(route.order) - 1)],
            insp._C_ROUTE))

    pcd_trk, ls_trk = insp._make_tracking_geom(trk, plan_pos)
    for g in (pcd_trk, ls_trk):
        if g is not None:
            geoms.append(g)

    lo, hi = scene.bounds
    extent = float(np.ptp(scene.facets.centers, axis=0).max())
    geoms.append(insp._make_ground_grid(
        float(lo[2]), lo, hi, step=max(0.5, round(extent / 10, 1))))

    return geoms


def capture(geoms, *, width: int, height: int, front, up, zoom: float,
            lookat=None, point_size: float = 9.0) -> np.ndarray:
    import open3d as o3d

    viz = o3d.visualization.Visualizer()
    viz.create_window(width=width, height=height, visible=False)
    for g in geoms:
        viz.add_geometry(g)

    ro = viz.get_render_option()
    ro.background_color = np.array([1.0, 1.0, 1.0])
    ro.point_size = point_size
    ro.line_width = 2.0
    ro.mesh_show_back_face = True
    ro.light_on = True

    vc = viz.get_view_control()
    vc.set_up(up)
    vc.set_front(front)
    if lookat is not None:
        vc.set_lookat(lookat)
    vc.set_zoom(zoom)

    for _ in range(5):
        viz.poll_events()
        viz.update_renderer()

    buf = viz.capture_screen_float_buffer(do_render=True)
    viz.destroy_window()
    return (np.asarray(buf) * 255.0 + 0.5).astype(np.uint8)


def autocrop(img: np.ndarray, pad_frac: float = 0.015) -> np.ndarray:
    non_white = np.any(img < 250, axis=2)
    if not non_white.any():
        return img
    ys, xs = np.where(non_white)
    y0, y1, x0, x1 = ys.min(), ys.max() + 1, xs.min(), xs.max() + 1
    img = img[y0:y1, x0:x1]
    pad = int(round(pad_frac * max(img.shape[:2])))
    if pad:
        img = np.pad(img, ((pad, pad), (pad, pad), (0, 0)),
                     constant_values=255)
    return img


def _resize_to_height(img: np.ndarray, h: int) -> np.ndarray:
    if img.shape[0] == h:
        return img
    from PIL import Image
    w = max(1, int(round(img.shape[1] * h / img.shape[0])))
    return np.asarray(Image.fromarray(img).resize((w, h), Image.LANCZOS))


def hconcat(imgs, gap_frac: float = 0.02) -> np.ndarray:
    h = min(im.shape[0] for im in imgs)
    imgs = [_resize_to_height(im, h) for im in imgs]
    gap = int(round(gap_frac * h))
    sep = np.full((h, gap, 3), 255, np.uint8)
    row = []
    for i, im in enumerate(imgs):
        if i:
            row.append(sep)
        row.append(im)
    return np.concatenate(row, axis=1)


def _font(px: int):
    from PIL import ImageFont
    return ImageFont.truetype(_FONT_PATH, px)


def _legend_items():
    pal = insp._distinct_colors(6)
    return [
        ("multi", pal, "Scan-Footprint (Farbe je Pose)"),
        ("swatch", insp._C_OVERLAP, "Registrierungs-Überlappung (≥ 2 Scans)"),
        ("swatch", insp._C_UNREACH, "unerreichbar (horizontale Fläche)"),
        ("line", insp._C_ROUTE, "Flugroute (TSP)"),
        ("line", insp._C_TRACKER, "Tracking-Standort + Sichtlinie"),
    ]


def draw_legend(width: int, font_px: int, items) -> np.ndarray:
    from PIL import Image, ImageDraw

    font = _font(font_px)
    sw = int(round(font_px * 1.15))
    gap_sym = int(round(font_px * 0.45))
    gap_item = int(round(font_px * 1.6))
    row_h = int(round(font_px * 1.9))
    pad_y = int(round(font_px * 0.6))

    probe = ImageDraw.Draw(Image.new("RGB", (1, 1)))

    def text_w(s):
        return int(probe.textlength(s, font=font))

    widths = [sw + gap_sym + text_w(txt) for _, _, txt in items]
    rows, cur, cur_w = [], [], 0
    for it, w in zip(items, widths):
        add = w + (gap_item if cur else 0)
        if cur and cur_w + add > width:
            rows.append((cur, cur_w)); cur, cur_w = [], 0
            add = w
        cur.append((it, w)); cur_w += add
    if cur:
        rows.append((cur, cur_w))

    H = pad_y * 2 + row_h * len(rows)
    canvas = Image.new("RGB", (width, H), (255, 255, 255))
    d = ImageDraw.Draw(canvas)

    for r, (row, row_w) in enumerate(rows):
        x = (width - row_w) // 2
        cy = pad_y + row_h * r + row_h // 2
        for (art, color, txt), w in row:
            top = cy - sw // 2
            if art == "line":
                col = tuple(int(c * 255) for c in color)
                d.line([(x, cy), (x + sw, cy)], fill=col,
                       width=max(3, font_px // 8))
                d.rectangle([x + sw // 2 - sw // 6, cy - sw // 6,
                             x + sw // 2 + sw // 6, cy + sw // 6], fill=col)
            elif art == "multi":
                n = len(color)
                cw = sw / n
                for k in range(n):
                    col = tuple(int(c * 255) for c in color[k])
                    d.rectangle([x + int(k * cw), top,
                                 x + int((k + 1) * cw), top + sw], fill=col)
                d.rectangle([x, top, x + sw, top + sw], outline=(60, 60, 60),
                            width=max(1, font_px // 22))
            else:
                col = tuple(int(c * 255) for c in color)
                d.rectangle([x, top, x + sw, top + sw], fill=col,
                            outline=(60, 60, 60), width=max(1, font_px // 22))
            tx = x + sw + gap_sym
            d.text((tx, cy), txt, font=font, fill=(20, 20, 20), anchor="lm")
            x += w + gap_item

    return np.asarray(canvas)


def _panel_label(img: np.ndarray, text: str, font_px: int) -> np.ndarray:
    from PIL import Image, ImageDraw
    im = Image.fromarray(img)
    d = ImageDraw.Draw(im)
    m = int(round(font_px * 0.5))
    d.text((m, m), text, font=_font(font_px), fill=(20, 20, 20), anchor="lt")
    return np.asarray(im)


def compose(panels, out_path: Path, *, labels=None) -> None:
    from PIL import Image

    if labels:
        pw = int(np.mean([p.shape[1] for p in panels]))
        lab_px = max(12, int(round(BODY_PT * pw / TEXTWIDTH_PT)))
        panels = [_panel_label(p, lab, lab_px) for p, lab in zip(panels, labels)]

    body = panels[0] if len(panels) == 1 else hconcat(panels)
    W = body.shape[1]
    font_px = max(12, int(round(BODY_PT * W / TEXTWIDTH_PT)))
    legend = draw_legend(W, font_px, _legend_items())
    final = np.concatenate([body, legend], axis=0)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(final).save(out_path)
    print(f"    -> {out_path}  ({final.shape[1]}x{final.shape[0]} px, "
          f"Legende {font_px}px ~ {BODY_PT:.0f}pt @ \\textwidth)")


def _load_scene(spec, cfg):
    if spec["kind"] == "synthetic":
        return scene_mod.SCENES[spec["name"]](cfg.resolution)
    return scene_mod.from_file(spec["mesh"], cfg.resolution,
                               assume_convex=spec.get("assume_convex", False),
                               name=spec["name"])


SCENES = {
    "box": dict(
        kind="synthetic", name="box", out="box_ilp.png",
        front=(0.6, -0.8, 0.35), up=(0, 0, 1), zoom=0.62, point_size=11.0),
    "notched_box": dict(
        kind="synthetic", name="notched_box", out="notched_box_ilp.png",
        front=(0.75, -0.55, 0.38), up=(0, 0, 1), zoom=0.6, point_size=10.0),
    "TestKorper1": dict(
        kind="mesh", name="TestKorper1", mesh="Meshes/TestKorper1.stl",
        out="TestKorper1_ilp.png",
        front=(0.75, -0.55, 0.38), up=(0, 0, 1), zoom=0.6, point_size=10.0),
}


def render_panel(key: str, cfg, *, width: int, height: int,
                 method: str) -> np.ndarray:
    spec = SCENES[key]
    print(f"[{key}] Szene laden ...")
    scene = _load_scene(spec, cfg)
    print(f"[{key}] Loesung berechnen ({method}) ...")
    poses, vis, selected, route, ov, trk = build_solution(scene, cfg, method)
    print(f"[{key}] {len(selected)} Posen, {len(ov.overlap_facets)} "
          f"Overlap-Facetten, {len(trk.stations)} Tracking-Standorte")
    geoms = mosaic_geometries(scene, poses, vis, selected, route, ov, trk, cfg)
    img = capture(geoms, width=width, height=height, front=spec["front"],
                  up=spec["up"], zoom=spec["zoom"],
                  point_size=spec.get("point_size", 9.0))
    return autocrop(img)


FIGURES = [
    ("box_ilp.png", ["box"], None),
    ("notched_testkorper_ilp.png", ["notched_box", "TestKorper1"],
     ["(a)", "(b)"]),
]


def main() -> None:
    ap = argparse.ArgumentParser(description="Evaluationsabbildungen rendern")
    ap.add_argument("--only", choices=list(SCENES), default=None,
                    help="nur eine Szene rendern (Einzelbild, ohne Montage)")
    ap.add_argument("--method", default="ilp", choices=["ilp", "greedy"])
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1440)
    ap.add_argument("--scratch", action="store_true",
                    help="in output/figtest statt LaTex/abbildung schreiben")
    args = ap.parse_args()

    cfg = load_config()
    out_dir = _SCRATCH if args.scratch else _FIG_DIR

    if args.only:
        panel = render_panel(args.only, cfg, width=args.width,
                             height=args.height, method=args.method)
        compose([panel], out_dir / SCENES[args.only]["out"])
        return

    cache = {}
    for fname, keys, labels in FIGURES:
        panels = [cache.setdefault(
                    k, render_panel(k, cfg, width=args.width,
                                    height=args.height, method=args.method))
                  for k in keys]
        compose(panels, out_dir / fname, labels=labels)


if __name__ == "__main__":
    main()
