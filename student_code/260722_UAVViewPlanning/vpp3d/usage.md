# vpp3d — Usage

Alle Befehle werden aus dem Verzeichnis `Code/` ausgeführt.

---

## Einstiegspunkte

| Modul | Zweck |
|---|---|
| `vpp3d.run` | Kompletter Pipeline-Durchlauf [1]–[7], erzeugt PNG |
| `vpp3d.inspector` | Interaktiver 3D-Viewer (Open3D), Tastatur-Navigation |
| `vpp3d.stepviz` | Schritt-für-Schritt Greedy-Ablauf (matplotlib) |

---

## `vpp3d.run` — Kompletter Durchlauf

```
python -m vpp3d.run [OPTIONEN]
```

### Parameter

| Flag | Typ | Default | Beschreibung |
|---|---|---|---|
| `--scene` | `box` \| `notched_box` | `box` | Synthetische Testszene |
| `--mesh PATH` | Pfad | — | Reales Mesh/Punktwolke statt Szene (STL, PLY) |
| `--assume-convex` | Flag | aus | Normalen radial nach außen (Punktwolken ohne Normalen) |
| `--resolution M` | float | aus config.toml | Facetten-Kantenlänge [m] überschreiben |
| `--method` | `greedy` \| `ilp` \| `both` | `greedy` | Set-Cover-Solver |
| `--k K` | int | `1` | k-Coverage: jede Facette von ≥ K Posen sehen lassen (Registrierungs-Redundanz); Bedarf auf erreichbare Abdeckung gekappt |
| `--baseline B` | float | aus config.toml | Kamera-Projektor-Basislinie [m]; `0` = Einzelsicht-Ablation |
| `--time-limit T` | float | `60.0` | ILP-Zeitlimit [s] |
| `--no-overlap` | Flag | aus | Overlap-Reparatur überspringen |
| `--no-tracking` | Flag | aus | Tracking-Standort-Planung überspringen |
| `--config PATH` | Pfad | `vpp3d/config.toml` | Alternative Konfigurationsdatei |
| `--show` | Flag | aus | Nach dem Lauf Open3D-Ansicht öffnen |

### Beispiele

```powershell
# Standard: Box-Szene, Greedy
python -m vpp3d.run

# Notched-Box mit Greedy + ILP-Vergleich
python -m vpp3d.run --scene notched_box --method both --time-limit 120

# Reales Mesh, nur Greedy, Open3D-Ansicht danach
python -m vpp3d.run --mesh Meshes/TestKorper1.stl --show

# Punktwolke (EasyCube), konvexe Normalenorientierung
python -m vpp3d.run --mesh Meshes/EasyCube.ply --assume-convex

# Ablation: Einzelsicht (kein Projektor-Kamera-Paar)
python -m vpp3d.run --baseline 0 --scene notched_box

# Feinere Facetten, ILP, kein Tracking
python -m vpp3d.run --resolution 0.15 --method ilp --no-tracking
```

Ausgabe: `vpp3d/output/<szene>_<methode>.png`

---

## `vpp3d.inspector` — Interaktiver 3D-Viewer

```
python -m vpp3d.inspector [OPTIONEN]
```

Führt dieselbe Pipeline wie `vpp3d.run` aus und öffnet anschließend einen
interaktiven Open3D-Viewer mit Tastatur-Navigation.

### Parameter

Identisch mit `vpp3d.run`, außer: kein `--no-tracking`, kein `--show`, kein
`--method both` (nur `greedy` oder `ilp`).

| Flag | Typ | Default | Beschreibung |
|---|---|---|---|
| `--scene` | `box` \| `notched_box` | `box` | Synthetische Testszene |
| `--mesh PATH` | Pfad | — | Reales Mesh/Punktwolke (STL, PLY) |
| `--assume-convex` | Flag | aus | Normalen-Orientierung für Punktwolken |
| `--resolution M` | float | aus config.toml | Facetten-Kantenlänge [m] |
| `--method` | `greedy` \| `ilp` | `greedy` | Set-Cover-Solver |
| `--k K` | int | `1` | k-Coverage: jede Facette von ≥ K Posen sehen lassen (Bedarf gekappt) |
| `--baseline B` | float | aus config.toml | Basislinie [m]; `0` = Einzelsicht |
| `--time-limit T` | float | `60.0` | ILP-Zeitlimit [s] |
| `--no-overlap` | Flag | aus | Overlap-Reparatur überspringen |
| `--config PATH` | Pfad | `vpp3d/config.toml` | Alternative Konfigurationsdatei |

### Tastaturkürzel im Viewer

| Taste | Modus / Aktion |
|---|---|
| `O` / `3` | **Übersicht** — alle Plan-Posen + TSP-Route; grün=abgedeckt, rot=offen, grau=unerreichbar |
| `C` / `1` | **Kandidaten-Modus** — alle Kandidatenposen durchsteppen; hellgrün=von dieser Pose sichtbar |
| `P` / `2` | **Plan-Modus** — nur gewählte Plan-Posen durchsteppen; FoV-Frustum + Footprint |
| `M` / `4` | **Mosaik** — jede Pose in eigener Farbe, ihr Footprint entsprechend; gelb=Registrierungs-Überlappung (≥2 Scans) |
| `N` / `→` | Nächste Pose (Kandidaten- und Plan-Modus) |
| `B` / `←` | Vorherige Pose (Kandidaten- und Plan-Modus) |
| `S` | Screenshot nach `vpp3d/output/inspector_<szene>_<modus>_<nr>.png` |
| `Q` / `Esc` | Schließen |

### Beispiele

```powershell
# Standard: Box-Szene, Greedy
python -m vpp3d.inspector

# Notched-Box mit ILP (längere Rechenzeit, weniger Posen)
python -m vpp3d.inspector --scene notched_box --method ilp --time-limit 120

# Reales Mesh
python -m vpp3d.inspector --mesh Meshes/TestKorper1.stl

# Feinere Auflösung, ohne Overlap-Reparatur
python -m vpp3d.inspector --resolution 0.15 --no-overlap
```

---

## `vpp3d.stepviz` — Greedy-Ablauf Schritt für Schritt

```
python -m vpp3d.stepviz [OPTIONEN]
```

Zeigt den Greedy-Set-Cover-Aufbau Pick für Pick: neu abgedeckte Facetten
(orange), Überlappung mit bereits abgedeckten (blau), fertig abgedeckte (grün).

### Parameter

| Flag | Typ | Default | Beschreibung |
|---|---|---|---|
| `--scene` | `box` \| `notched_box` | `notched_box` | Synthetische Testszene |
| `--mesh PATH` | Pfad | — | Reales Mesh/Punktwolke (STL, PLY) |
| `--assume-convex` | Flag | aus | Normalen-Orientierung für Punktwolken |
| `--resolution M` | float | aus config.toml | Facetten-Kantenlänge [m] |
| `--baseline B` | float | aus config.toml | Basislinie [m] |
| `--config PATH` | Pfad | `vpp3d/config.toml` | Alternative Konfigurationsdatei |
| `--show` | Flag | aus | Interaktiver Navigator statt PNG-Export |

### Beispiele

```powershell
# PNGs je Greedy-Schritt nach vpp3d/output/steps_notched_box_greedy/
python -m vpp3d.stepviz --scene notched_box

# Interaktiv navigieren (Pfeiltasten, n/b, s=Screenshot, q=schließen)
python -m vpp3d.stepviz --scene notched_box --show

# Reales Mesh, interaktiv
python -m vpp3d.stepviz --mesh Meshes/TestKorper1.stl --show
```

---

## `vpp3d.stepfigs` — Folienbilder der Kandidatenkonstruktion

```
python -m vpp3d.stepfigs [OPTIONEN]
```

Rendert je Konstruktions-/Verwurfsschritt ein PNG **ohne Beschriftung** nach
`vpp3d/output/praesentation/` — alle mit identischer Kamera, also als Folge
überblendbar:

| # | Datei | Inhalt |
|---|---|---|
| 1 | `…_1_facetten.png` | Facetten-Mittelpunkte (Abtastziele) |
| 2 | `…_2_normalen.png` | Außennormale **jeder** Facette als Pfeil (orange) |
| 3 | `…_3_kegelsampling.png` | Inzidenzkegel einer Facette + Kandidatenposen (grün) |
| 4 | `…_4_dedup.png` | redundante Posen blass, Repräsentant je Rasterzelle grün |
| 5 | `…_5_abstand.png` | Sicherheits-/Bodenabstand: verworfen rot, zulässig grün |
| 6 | `…_6_dualsicht.png` | eine Pose: dual gesehen grün, nur vom Projektor rot |

`--resolution` ist hier auf **0,08 m** vorbelegt (feine Facetten fürs Bild, nicht
aus `config.toml`). Punktgrößen und Pfeillängen skalieren mit; die Rohposen­wolken
(Bild 4/5) werden für die Darstellung ausgedünnt — die Zahlen auf der Konsole
bleiben vollständig.

Weitere Parameter neben denen von `stepviz`: `--only N [N …]` (nur einzelne
Bilder), `--width/--height` (Renderauflösung), `--crop` (fester Mittenausschnitt,
`1.0` = ganzes Bild), `--out` (Zielverzeichnis).

```powershell
python -m vpp3d.stepfigs                          # alle sechs, notched_box
python -m vpp3d.stepfigs --scene box --only 3 6
```

---

## `vpp3d/config.toml` — Konfiguration

Zentrale Parameterdatei. Alle Winkel in Grad, Distanzen in Metern.
Mit `--config PATH` kann eine alternative Datei übergeben werden.

```toml
[working_distance]
d_min = 2.8          # Nah-Grenze des Arbeitsabstands [m]
d_max = 3.2          # Fern-Grenze [m]
# d_opt = 3.0        # Soll-Standoff; auskommentiert → Mittelwert

[fov]
fov_h_deg = 40.0     # Horizontales Sichtfeld [°]
fov_v_deg = 30.0     # Vertikales Sichtfeld [°]

[sensor]
baseline = 0.4       # Projektor-Kamera-Basislinie [m]; 0 = Einzelsicht

[incidence]
theta_max_deg = 60.0 # Max. Einfallswinkel Sichtstrahl/Normale [°]

[orientation]
pitch_min_deg = -10.0  # Gimbal-Untergrenze [°]
pitch_max_deg =  10.0  # Gimbal-Obergrenze [°]

[drone]
safety_distance = 1.5  # Mindestabstand Drohne/Objekt [m]

[sampling]
n_azimuth = 8          # Azimut-Richtungen je Kegel-Ring
cone_fractions = [0.0, 0.5, 0.85]  # Kegel-Ringe als Bruchteil von theta_max
n_distance = 2         # Distanz-Stützstellen in [d_min, d_max]
yaw_bin_deg = 15.0     # Dedup-Raster Yaw [°]
pitch_bin_deg = 10.0   # Dedup-Raster Pitch [°]
resolution = 0.25      # Facetten-Soll-Kantenlänge [m]
# voxel = 0.45         # Dedup-Ortsraster [m]; auskommentiert → 0.15 * d_opt

[registration]
min_overlap = 0.25     # Overlap-Schwelle: gemeinsame Fläche (kleinerer Scan)

[tracking]
range_min = 1.0        # Min. Tracker-Drohne-Abstand [m]
range_max = 8.0        # Max. Tracker-Drohne-Abstand [m]
grid_step = 0.5        # Rasterauflösung für Tracking-Standorte [m]
margin = 2.0           # Raster-Rand über Posen-Hülle [m]
```

---

## Schnellreferenz

```powershell
# Einfachster Start
python -m vpp3d.run

# Greedy vs. ILP vergleichen
python -m vpp3d.run --method both --time-limit 60

# Interaktiv erkunden
python -m vpp3d.inspector --scene notched_box

# Greedy-Ablauf animieren
python -m vpp3d.stepviz --show
```
