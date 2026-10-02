#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Erzeugt die Abbildungen der Masterarbeit ohne laufendes ROS-System.

  abbildung/karte_eingang.png, karte_kanten.png,      -> fig:karte_pipeline
  abbildung/karte_belegung.png, karte_distanzfeld.png
  abbildung/drohnenposen.png                          -> fig:drohnenposen
  abbildung/ergebnis_s1.png ... ergebnis_s4.png       -> fig:eval_ergebnisse
  abbildung/ergebnis_legende.png                      -> gemeinsame Legende dazu

Die Kartenkette und die Erzeugung der Drohnenposen sind 1:1 aus
image_to_ros.py übernommen. Die Gruppierung entspricht optimizer_node.py
(K-Means mit random_state=42). Die Formationen stammen aus der CSV-Datei
der Hauptauswertung.

Aufruf (Beispiele):
  python3 abbildungen_erstellen.py                      # alles, CSV im Standardpfad
  python3 abbildungen_erstellen.py --nur karte posen    # ohne Ergebnisbilder
  python3 abbildungen_erstellen.py --csv ~/map_ws/ergebnisse/haupt_gesamt.csv
  python3 abbildungen_erstellen.py --format pdf         # Vektorgrafik statt PNG
"""

import argparse
import csv
import json
import math
import os
import statistics

import cv2
import numpy as np
import scipy.ndimage as ndimage
from sklearn.cluster import KMeans

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch, Wedge
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ---------------------------------------------------------------------------
# Parameter (identisch zu image_to_ros.py und optimizer_node.py)
# ---------------------------------------------------------------------------
PIXEL_TO_METER = 0.1
BUILDING_HEIGHT = 10.0
STANDOFF_DISTANCE_M = 2.5
TARGET_RESOLUTION_M = 2.5
Z_LEVELS = [1.25, 3.75, 6.25, 8.75]
D_SAFE = 1.0

# ---------------------------------------------------------------------------
# Darstellung (serifenlose Schrift wie im Dokument)
# ---------------------------------------------------------------------------
plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["Arial", "Helvetica", "Liberation Sans", "DejaVu Sans"],
    "font.size": 8,
    "axes.labelsize": 8,
    "xtick.labelsize": 7,
    "ytick.labelsize": 7,
    "legend.fontsize": 7,
    "axes.linewidth": 0.6,
    "savefig.dpi": 300,
    "axes.formatter.use_locale": False,
})
FARBE_OBJEKT = "0.25"      # Messobjekt
FARBE_HINDERNIS = "0.62"   # weitere Hindernisse
FARBE_KONTUR = "#1f77b4"   # Iso-Kontur
GRUPPENFARBEN = plt.get_cmap("tab10").colors


def komma(x, pos=None):
    """Achsenbeschriftung mit Dezimalkomma."""
    s = f"{x:g}"
    return s.replace(".", ",").replace("-", "\u2212")


# ===========================================================================
# 1. Kartenkette (aus image_to_ros.py)
# ===========================================================================
def szenario_bild(mode):
    """Synthetische Szenariokarte, identisch zu generate_hardcoded_scenario()."""
    img = np.ones((200, 200), dtype=np.uint8) * 255
    if mode == 1:
        cv2.rectangle(img, (80, 80), (120, 120), 0, -1)
    elif mode == 2:
        cv2.rectangle(img, (80, 80), (120, 120), 0, -1)
        cv2.rectangle(img, (40, 150), (160, 160), 0, -1)
    elif mode == 3:
        cv2.rectangle(img, (50, 60), (70, 140), 0, -1)
        cv2.rectangle(img, (50, 120), (150, 140), 0, -1)
        cv2.rectangle(img, (130, 60), (150, 140), 0, -1)
    elif mode == 4:
        cv2.rectangle(img, (50, 60), (70, 140), 0, -1)
        cv2.rectangle(img, (50, 120), (150, 140), 0, -1)
        cv2.rectangle(img, (130, 60), (150, 140), 0, -1)
        cv2.rectangle(img, (30, 180), (170, 190), 0, -1)
        cv2.line(img, (10, 20), (40, 50), 0, thickness=8)
        cv2.line(img, (190, 20), (160, 50), 0, thickness=8)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def kartenkette(img):
    """Führt die Verarbeitungskette aus und gibt alle Zwischenergebnisse zurück."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 30, 100)
    kernel = np.ones((7, 7), np.uint8)
    closing = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=3)
    found, _ = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in found if cv2.contourArea(c) > 100]
    if not contours:
        raise RuntimeError("Keine Konturen gefunden.")

    h, w = img.shape[:2]
    cx, cy = w // 2, h // 2
    ziel, dmin = 0, float("inf")
    for idx, cnt in enumerate(contours):
        m = cv2.moments(cnt)
        if m["m00"] != 0:
            # int()-Rundung wie im Original
            d = math.hypot(int(m["m10"] / m["m00"]) - cx, int(m["m01"] / m["m00"]) - cy)
            if d < dmin:
                dmin, ziel = d, idx

    belegt = np.zeros((h, w), dtype=np.uint8)
    cv2.drawContours(belegt, contours, -1, 255, thickness=cv2.FILLED)
    belegt_welt = np.flipud(belegt) > 0              # Zeile 0 = kleinstes y

    ziel_maske = np.zeros((h, w), dtype=np.uint8)
    cv2.drawContours(ziel_maske, [contours[ziel]], -1, 255, thickness=cv2.FILLED)
    ziel_welt = np.flipud(ziel_maske) > 0

    # Distanzfeld wie im Optimierungsknoten (Meter)
    esdf = ndimage.distance_transform_edt(~belegt_welt) * PIXEL_TO_METER

    return {
        "bild": img, "gray": gray, "kanten": edges, "geschlossen": closing,
        "konturen": contours, "ziel": ziel, "h": h, "w": w, "cx": cx, "cy": cy,
        "belegt_welt": belegt_welt, "ziel_welt": ziel_welt, "esdf": esdf,
        "origin_x": -cx * PIXEL_TO_METER, "origin_y": -cy * PIXEL_TO_METER,
    }


def ausdehnung(k):
    """Weltkoordinaten der Kartenränder für imshow(origin='lower')."""
    x0, y0 = k["origin_x"], k["origin_y"]
    return [x0, x0 + k["w"] * PIXEL_TO_METER, y0, y0 + k["h"] * PIXEL_TO_METER]


def drohnenposen(k):
    """Vorläufige Drohnenposen wie in image_to_ros.py (Reihenfolge identisch)."""
    res = PIXEL_TO_METER
    cnt = k["konturen"][k["ziel"]]
    umfang = cv2.arcLength(cnt, True) * res
    n_saeulen = max(1, int(umfang / TARGET_RESOLUTION_M))

    d_ziel = ndimage.distance_transform_edt(~k["ziel_welt"]) * res
    tol = res / 2.0
    ring = (d_ziel >= STANDOFF_DISTANCE_M - tol) & (d_ziel <= STANDOFF_DISTANCE_M + tol)

    # argwhere liefert zeilenweise Reihenfolge wie die verschachtelten Schleifen
    idx = np.argwhere(ring)
    punkte = np.column_stack([
        k["origin_x"] + (idx[:, 1] + 0.5) * res,
        k["origin_y"] + (idx[:, 0] + 0.5) * res,
    ])
    km = KMeans(n_clusters=n_saeulen, random_state=42, n_init=10).fit(punkte)

    xb, yb, bw, bh = cv2.boundingRect(cnt)
    tcx = ((xb + bw / 2.0) - k["cx"]) * res
    tcy = (k["cy"] - (yb + bh / 2.0)) * res

    saeulen, posen = [], []
    for c in km.cluster_centers_:
        ci = int(math.floor((c[1] - k["origin_y"]) / res))
        cj = int(math.floor((c[0] - k["origin_x"]) / res))
        if 0 <= ci < k["h"] and 0 <= cj < k["w"] and k["belegt_welt"][ci, cj]:
            continue                                   # Säule im Hindernis
        yaw = math.atan2(tcy - c[1], tcx - c[0])
        saeulen.append([c[0], c[1], yaw])
        for z in Z_LEVELS:
            posen.append([c[0], c[1], z, yaw])

    return {
        "umfang": umfang, "punkte": punkte, "d_ziel": d_ziel,
        "saeulen": np.array(saeulen), "posen": np.array(posen),
    }


def gruppierung(posen, K):
    """Gruppierung wie in optimizer_node.py."""
    if K == 1:
        return np.zeros(len(posen), dtype=int)
    km = KMeans(n_clusters=K, random_state=42, n_init=10).fit(posen[:, :3])
    return km.labels_


def welt_polygon(k, cnt, eps=1.0):
    """Kontur (Pixel) als vereinfachtes Polygon in Weltkoordinaten."""
    approx = cv2.approxPolyDP(cnt, eps, True)[:, 0, :].astype(float)
    x = (approx[:, 0] + 0.5 - k["cx"]) * PIXEL_TO_METER
    y = (k["h"] - approx[:, 1] - 0.5 - k["cy"]) * PIXEL_TO_METER
    return np.column_stack([x, y])


def karte_zeichnen(ax, k, sicherheitszone=False):
    """Hindernisse in Draufsicht; Messobjekt dunkel, übrige Hindernisse hell."""
    ext = ausdehnung(k)
    rgba = np.ones(k["belegt_welt"].shape + (4,))
    rgba[..., 3] = 0.0
    if sicherheitszone:
        zone = (~k["belegt_welt"]) & (k["esdf"] < D_SAFE)
        rgba[zone] = matplotlib.colors.to_rgba("#f4c7c3", 0.7)
    rgba[k["belegt_welt"]] = matplotlib.colors.to_rgba(FARBE_HINDERNIS)
    rgba[k["ziel_welt"]] = matplotlib.colors.to_rgba(FARBE_OBJEKT)
    ax.imshow(rgba, origin="lower", extent=ext, interpolation="nearest", zorder=1)
    ax.set_xlim(ext[0], ext[1])
    ax.set_ylim(ext[2], ext[3])
    ax.set_aspect("equal")
    ax.xaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(komma))
    ax.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(komma))
    ax.set_xticks([-10, -5, 0, 5, 10])
    ax.set_yticks([-10, -5, 0, 5, 10])


# ===========================================================================
# 2. Abbildung Kartenkette (vier Einzelbilder gleicher Größe)
# ===========================================================================
def abb_kartenkette(k, ordner, fmt):
    ext = ausdehnung(k)

    def neue_figur():
        fig = plt.figure(figsize=(1.6, 1.95))
        ax = fig.add_axes([0.02, 0.22, 0.96, 0.76])
        cax = fig.add_axes([0.12, 0.13, 0.76, 0.04])
        for a in (ax,):
            a.set_xticks([]); a.set_yticks([])
            for s in a.spines.values():
                s.set_linewidth(0.6)
        cax.set_visible(False)
        return fig, ax, cax

    # a) Eingangsbild
    fig, ax, _ = neue_figur()
    ax.imshow(cv2.cvtColor(k["bild"], cv2.COLOR_BGR2RGB), extent=ext, interpolation="nearest")
    fig.savefig(os.path.join(ordner, f"karte_eingang.{fmt}"))
    plt.close(fig)

    # b) Kanten (Canny, schwarz auf weiß)
    fig, ax, _ = neue_figur()
    ax.imshow(255 - k["kanten"], cmap="gray", extent=ext, interpolation="nearest", vmin=0, vmax=255)
    fig.savefig(os.path.join(ordner, f"karte_kanten.{fmt}"))
    plt.close(fig)

    # c) Belegungsraster (binär)
    fig, ax, _ = neue_figur()
    ax.imshow(~k["belegt_welt"], cmap="gray", origin="lower", extent=ext,
              interpolation="nearest", vmin=0, vmax=1)
    fig.savefig(os.path.join(ordner, f"karte_belegung.{fmt}"))
    plt.close(fig)

    # d) Distanzfeld mit Farbskala
    fig, ax, cax = neue_figur()
    esdf = np.ma.masked_where(k["belegt_welt"], k["esdf"])
    cmap = plt.get_cmap("viridis").copy()
    cmap.set_bad("black")
    im = ax.imshow(esdf, cmap=cmap, origin="lower", extent=ext, interpolation="nearest")
    ax.contour(k["esdf"], levels=[D_SAFE], colors="white", linewidths=0.5,
               linestyles="--", origin="lower", extent=ext)
    cax.set_visible(True)
    cb = fig.colorbar(im, cax=cax, orientation="horizontal")
    cb.ax.tick_params(labelsize=6, length=2, pad=1)
    cb.ax.xaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(komma))
    cb.set_label("$d$ in m", fontsize=6, labelpad=1)
    fig.savefig(os.path.join(ordner, f"karte_distanzfeld.{fmt}"))
    plt.close(fig)
    print("  Kartenkette: 4 Einzelbilder gespeichert")


# ===========================================================================
# 3. Abbildung Drohnenposen (Draufsicht und Schrägansicht)
# ===========================================================================
def abb_drohnenposen(k, p, szenario, ordner, fmt):
    fig = plt.figure(figsize=(6.3, 3.0))

    # --- Draufsicht ---------------------------------------------------------
    ax = fig.add_axes([0.07, 0.13, 0.40, 0.80])
    karte_zeichnen(ax, k)
    ext = ausdehnung(k)
    ax.contour(p["d_ziel"], levels=[STANDOFF_DISTANCE_M], colors=FARBE_KONTUR,
               linewidths=0.8, origin="lower", extent=ext, zorder=2)
    s = p["saeulen"]
    ax.quiver(s[:, 0], s[:, 1], np.cos(s[:, 2]), np.sin(s[:, 2]), angles="xy",
              scale_units="xy", scale=1.0 / 0.9, width=0.006, color="0.1", zorder=4)
    ax.scatter(s[:, 0], s[:, 1], s=14, color="#d62728", edgecolor="k",
               linewidth=0.4, zorder=5)
    ax.set_xlabel("$x$ in m")
    ax.set_ylabel("$y$ in m")
    ax.set_title("Draufsicht", fontsize=8)

    # --- Schrägansicht --------------------------------------------------------
    ax3 = fig.add_axes([0.50, 0.02, 0.50, 0.96], projection="3d")
    for idx, cnt in enumerate(k["konturen"]):
        poly = welt_polygon(k, cnt)
        farbe = FARBE_OBJEKT if idx == k["ziel"] else FARBE_HINDERNIS
        alpha = 0.55 if idx == k["ziel"] else 0.25
        flaechen = []
        n = len(poly)
        for i in range(n):
            a, b = poly[i], poly[(i + 1) % n]
            flaechen.append([(a[0], a[1], 0), (b[0], b[1], 0),
                             (b[0], b[1], BUILDING_HEIGHT), (a[0], a[1], BUILDING_HEIGHT)])
        flaechen.append([(q[0], q[1], BUILDING_HEIGHT) for q in poly])
        ax3.add_collection3d(Poly3DCollection(flaechen, facecolor=farbe, alpha=alpha * 0.7,
                                              edgecolor="0.3", linewidth=0.3))
    # Iso-Kontur am Boden
    cs = plt.figure().add_subplot().contour(p["d_ziel"], levels=[STANDOFF_DISTANCE_M],
                                            origin="lower", extent=ext)
    for seg in cs.allsegs[0]:
        ax3.plot(seg[:, 0], seg[:, 1], 0, color=FARBE_KONTUR, linewidth=0.8)
    plt.close(cs.axes.figure)
    # Messsäulen
    for x, y, _ in s:
        ax3.plot([x, x], [y, y], [0, Z_LEVELS[-1]], color="0.5", linewidth=0.4, linestyle=":")
    P = p["posen"]
    ax3.scatter(P[:, 0], P[:, 1], P[:, 2], s=6, color="#d62728", edgecolor="k",
                linewidth=0.3, depthshade=False)
    ax3.set_xlim(-10, 10); ax3.set_ylim(-10, 10); ax3.set_zlim(0, BUILDING_HEIGHT)
    ax3.set_box_aspect((20, 20, 10))
    ax3.view_init(elev=28, azim=-58)
    ax3.set_xlabel("$x$ in m", labelpad=-6)
    ax3.set_ylabel("$y$ in m", labelpad=-6)
    ax3.set_zlabel("$z$ in m", labelpad=-7)
    ax3.tick_params(pad=-3, labelsize=6)
    ax3.set_xticks([-5, 0, 5]); ax3.set_yticks([-5, 0, 5]); ax3.set_zticks([0, 5, 10])
    ax3.set_title("Schrägansicht", fontsize=8, y=0.93)

    legende = [
        Patch(facecolor=FARBE_OBJEKT, label="Messobjekt"),
        Line2D([], [], color=FARBE_KONTUR, label=f"Iso-Kontur ($d_{{\\mathrm{{scan}}}}$ = "
               f"{komma(STANDOFF_DISTANCE_M)} m)"),
        Line2D([], [], marker="o", linestyle="", markerfacecolor="#d62728",
               markeredgecolor="k", markersize=4, label="Messsäule / Drohnenpose"),
    ]
    if len(k["konturen"]) > 1:
        legende.insert(1, Patch(facecolor=FARBE_HINDERNIS, label="weiteres Hindernis"))
    fig.legend(handles=legende, loc="upper center", ncol=len(legende), frameon=False,
               bbox_to_anchor=(0.5, 0.0))
    datei = os.path.join(ordner, f"drohnenposen.{fmt}")
    fig.savefig(datei, bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)
    print(f"  Drohnenposen: Szenario {szenario}, {len(s)} Messsäulen, "
          f"{len(P)} Posen, Umfang {p['umfang']:.1f} m")


# ===========================================================================
# 4. Ergebnisbilder der Hauptauswertung
# ===========================================================================
def lies_csv(pfad):
    with open(pfad, newline="") as f:
        return list(csv.DictReader(f))


def median_lauf(zeilen, szenario, typen, w_move, w_obs):
    kandidaten = []
    for z in zeilen:
        try:
            kosten = float(z["Finale_Kosten_J"])
        except (ValueError, KeyError):
            continue
        if (z.get("Messobjekt") == f"Szenario_{szenario}_Final"
                and z.get("robot_types") == typen
                and abs(float(z["w_move"]) - w_move) < 1e-9
                and abs(float(z["w_obs"]) - w_obs) < 1e-9
                and z.get("Status") in ("abbruchkriterium", "max_k")
                and math.isfinite(kosten)):
            kandidaten.append((kosten, z))
    if not kandidaten:
        return None, 0
    kandidaten.sort(key=lambda t: t[0])
    ziel = statistics.median_low([c for c, _ in kandidaten])
    for c, z in kandidaten:
        if c == ziel:
            return z, len(kandidaten)


def viertelkreis(ax, x, y, farben, r=0.30):
    """Messsäule als Kreis aus vier Vierteln; jedes Viertel = eine Höhe."""
    for i, f in enumerate(farben):
        ax.add_patch(Wedge((x, y), r, 90 * i, 90 * (i + 1), facecolor=f,
                           edgecolor="0.3", linewidth=0.25, zorder=4))


def abb_ergebnis(szenario, zeile, n_laeufe, ordner, fmt):
    k = kartenkette(szenario_bild(szenario))
    p = drohnenposen(k)
    K = int(zeile["Gewaehltes_k"])
    labels = gruppierung(p["posen"], K)
    formationen = json.loads(zeile["Formationen"])
    groessen_csv = json.loads(zeile["Gruppengroessen"])
    typen = zeile["robot_types"]

    groessen = [int(np.sum(labels == i)) for i in range(K)]
    if groessen != groessen_csv:
        print(f"  WARNUNG Szenario {szenario}: Gruppengrößen {groessen} weichen von der "
              f"CSV-Datei {groessen_csv} ab. Vermutlich andere scikit-learn-Version; "
              "die Zuordnung Gruppe <-> Formation ist dann nicht gesichert.")

    fig, ax = plt.subplots(figsize=(3.1, 3.1))
    karte_zeichnen(ax, k, sicherheitszone=True)

    # Drohnenposen: je Messsäule vier Viertel (von unten nach oben gegen den Uhrzeigersinn)
    P = p["posen"]
    for i in range(0, len(P), len(Z_LEVELS)):
        farben = [GRUPPENFARBEN[labels[i + j] % 10] for j in range(len(Z_LEVELS))]
        viertelkreis(ax, P[i, 0], P[i, 1], farben)

    # Roboter
    for g, form in enumerate(formationen):
        farbe = GRUPPENFARBEN[g % 10]
        for r in range(len(form) // 2):
            typ = typen[r] if r < len(typen) else "A"
            ax.scatter(form[2 * r], form[2 * r + 1], s=42 if typ == "A" else 32,
                       marker="^" if typ == "A" else "s", color=farbe,
                       edgecolor="k", linewidth=0.8, zorder=6)

    ax.set_xlabel("$x$ in m")
    ax.set_ylabel("$y$ in m")
    ax.set_title(f"Szenario {szenario}: $K^{{*}}$ = {K}", fontsize=8)

    fig.savefig(os.path.join(ordner, f"ergebnis_s{szenario}.{fmt}"),
                bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)
    print(f"  Ergebnis Szenario {szenario}: Median von {n_laeufe} Läufen, "
          f"K* = {K}, C_ges = {float(zeile['Finale_Kosten_J']):.2f}, Run {zeile['Run_ID']}")


def abb_ergebnis_legende(ordner, fmt):
    """Gemeinsame Legende als eigenes Bild unter den vier Ergebnisbildern."""
    h = [
        Patch(facecolor=FARBE_OBJEKT, label="Messobjekt"),
        Patch(facecolor=FARBE_HINDERNIS, label="Hindernis"),
        Patch(facecolor="#f4c7c3", label="Abstand $< d_{\\mathrm{safe}}$"),
        Line2D([], [], marker="o", linestyle="", markerfacecolor="0.75",
               markeredgecolor="0.3", markersize=6,
               label="Messsäule (Viertel = Höhenstufe)"),
        Line2D([], [], marker="^", linestyle="", markerfacecolor="0.75",
               markeredgecolor="k", markersize=6, label="Roboter Typ A"),
        Line2D([], [], marker="s", linestyle="", markerfacecolor="0.75",
               markeredgecolor="k", markersize=5, label="Roboter Typ B"),
    ]
    fig = plt.figure(figsize=(6.3, 0.35))
    fig.legend(handles=h, loc="center", ncol=6, frameon=False, fontsize=7,
               handlelength=1.2, columnspacing=1.2, handletextpad=0.4)
    fig.savefig(os.path.join(ordner, f"ergebnis_legende.{fmt}"),
                bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)
    print("  Gemeinsame Legende gespeichert")


# ===========================================================================
# Hauptprogramm
# ===========================================================================
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--nur", nargs="+", choices=["karte", "posen", "ergebnisse"],
                    default=["karte", "posen", "ergebnisse"])
    ap.add_argument("--ausgabe", default="abbildung", help="Zielordner (Standard: abbildung)")
    ap.add_argument("--format", default="png", choices=["png", "pdf"])
    ap.add_argument("--szenario-karte", type=int, default=4,
                    help="Szenario für die Kartenkette (Standard: 4)")
    ap.add_argument("--luftbild", default=None,
                    help="Statt eines Szenarios ein eigenes Bild für die Kartenkette")
    ap.add_argument("--szenario-posen", type=int, default=3,
                    help="Szenario für die Drohnenposen (Standard: 3)")
    ap.add_argument("--csv", default=os.path.expanduser("~/map_ws/ergebnisse/haupt_gesamt.csv"))
    ap.add_argument("--typen", default="AAAABB", help="Flotte der Hauptauswertung")
    ap.add_argument("--w-move", type=float, default=1.0)
    ap.add_argument("--w-obs", type=float, default=1.0)
    args = ap.parse_args()

    os.makedirs(args.ausgabe, exist_ok=True)

    if "karte" in args.nur:
        if args.luftbild:
            img = cv2.imread(args.luftbild)
            if img is None:
                raise SystemExit(f"Bild nicht lesbar: {args.luftbild}")
        else:
            img = szenario_bild(args.szenario_karte)
        abb_kartenkette(kartenkette(img), args.ausgabe, args.format)

    if "posen" in args.nur:
        k = kartenkette(szenario_bild(args.szenario_posen))
        abb_drohnenposen(k, drohnenposen(k), args.szenario_posen, args.ausgabe, args.format)

    if "ergebnisse" in args.nur:
        if not os.path.isfile(args.csv):
            print(f"  CSV-Datei nicht gefunden: {args.csv} -> Ergebnisbilder übersprungen")
        else:
            zeilen = lies_csv(args.csv)
            for s in (1, 2, 3, 4):
                z, n = median_lauf(zeilen, s, args.typen, args.w_move, args.w_obs)
                if z is None:
                    print(f"  Szenario {s}: keine passenden Läufe in der CSV-Datei")
                    continue
                abb_ergebnis(s, z, n, args.ausgabe, args.format)
            abb_ergebnis_legende(args.ausgabe, args.format)


if __name__ == "__main__":
    main()