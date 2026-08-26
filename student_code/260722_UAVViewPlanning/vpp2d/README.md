# vpp2d — 2D-Prototyp des alternativen View-Planning-Ansatzes

Eigenständige, **vom bestehenden 3D-Code (`../pipeline`, `../*.py`) bewusst
getrennte** Implementierung des alternativen Entwurfs („Empfohlene Pipeline,
offline VPP"). Sie validiert den Ansatz in einer 2D-Umgebung, die in Sekunden
debugbar ist — vgl. die Empfehlung „erst 2D-Prototyp" in `../PIPELINE.md`.

In 2D kollabiert die 4-DoF-Kinematik (x, y, z, yaw) auf 3 DoF (x, y, yaw); z und
Pitch entfallen. **Die Constraint-Struktur — Inzidenzkegel, Arbeitsschale, FoV,
Occlusion, duale Sicht — bleibt identisch** und wird hier vollständig
durchgerechnet.

> **Fokus: reine Sichtlinie (line of sight).** Kein Tracking-/Standortmodell —
> Repositionierung der mobilen Tracking-Einheit wird *nicht* optimiert (für diese
> Arbeit irrelevant). Es gibt daher keinen Arbeitskreis und kein bi-level set
> cover; das Set Cover ist single-level über die Sichtbarkeitsmatrix.

## Pipeline (Spiegel des Bild-Entwurfs, in 2D)

| Schritt | Modul | Box im Entwurf |
|---|---|---|
| [1] Facetten als Primitiv | `scene.py` | Mesh-Facetten als Primitiv |
| [2] Posenregion je Facette samplen | `candidates.sample_pose_regions` | Posenregion pro Facette samplen |
| [3] Projektion auf Posenraum + Dedup | `candidates.project_and_dedup` | **Projektion auf 4-DoF-Raum** |
| [4] Coverage-Matrix per Raycast, duale Sicht | `visibility.py` | Coverage-Matrix per Raycasting |
| [6] Set Cover (Greedy + ILP) | `setcover.py` | (clustered) set cover → single-level |
| [7] TSP-Tour | `sequencing.py` | TSP-Tour |

### Differenzierende Elemente

1. **Facetten** als Abdeckungsziel (Polygon-Segmente mit Außennormalen) statt
   Poisson-gesampelter Patch-Punktwolke.
2. **Projektion + Dedup** als expliziter eigener Schritt
   (`candidates.project_and_dedup`): die pro Facette gesampelten Posen werden
   auf den von der Drohne stellbaren Raum (x, y, yaw; Roll gesperrt, in 3D
   zusätzlich Pitch geklemmt) projiziert und über ein Ortsvoxel- × Yaw-Bin-Raster
   dedupliziert. Benachbarte Facetten teilen denselben Posenraum → der Dedup
   kollabiert die hochredundante Rohmenge (typ. Faktor ~2 in 2D, in 3D höher).
3. **Duale Sicht** (`visibility.py`): ein Scan gilt nur, wenn Projektor *und*
   Kamera (um die Basislinie versetzt) die Facette unverdeckt und im FoV sehen.
   Die bestehende `pipeline/visibility.py` macht nur einen einzelnen Raycast.
   `baseline = 0` reduziert das Modell auf die klassische Einzelsicht (Ablation).

## Aufruf

Aus dem Verzeichnis `Code/` (venv mit numpy/scipy/matplotlib/ortools):

```powershell
.venv\Scripts\python -m vpp2d.run                                   # box, Greedy
.venv\Scripts\python -m vpp2d.run --scene notched_box --method both # Greedy vs. ILP
.venv\Scripts\python -m vpp2d.run --stl Meshes/TestKorper1.stl --method both   # realer Mesh-Schnitt
.venv\Scripts\python -m vpp2d.run --scene notched_box --baseline 0  # Ablation: Einzelsicht statt dual
```

Ergebnis-PNGs (4-Panel-Übersicht: Facetten+Normalen, Kandidatenposen,
Coverage-Grad, Lösung mit gewählten Posen + Route) landen in `vpp2d/output/`.

### Schritt-für-Schritt-Debug-Viewer

Zeigt den Greedy-Aufbau der Abdeckung Pick für Pick (Schritt 0 = nichts erfasst):

```powershell
.venv\Scripts\python -m vpp2d.stepviz --scene notched_box           # PNG-Frames
.venv\Scripts\python -m vpp2d.stepviz --scene notched_box --show     # interaktiv
.venv\Scripts\python -m vpp2d.stepviz --stl Meshes/TestKorper1.stl
```

- `--save` (Default): je Schritt ein PNG nach `output/steps_<szene>_greedy/step_000.png …`.
- `--show`: interaktiver Navigator — **→ / n** vor, **← / b** zurück, **Home/End**,
  **s** speichern, **q** schließen.

Pro Frame sichtbar: bereits abgedeckte Facetten (grün), noch offene (rot),
unerreichbare (grau); die in diesem Schritt gewählte Pose mit Blickrichtung und
FoV-Footprint (Arbeitsschalen-Sektor); die **neu** erfassten Facetten (orange)
und die **Überlappung** mit bereits Erfasstem (blau). Der Titel führt Fortschritt
(`x/n erfasst`) und Posenzahl mit.

### Testszenen

- `box` — konvexes 4×4-m-Quadrat, 2D-Analogon zu **EasyCube** (alles frei
  einsehbar).
- `notched_box` — 4,4×3,1-m-Rechteck mit rechteckiger Tasche, 2D-Analogon zu
  **TestKorper1**: erzeugt Selbstverdeckung (die duale Sicht verwirft Posen, bei
  denen die Kamera durch die Taschenkante verdeckt wird) und unerreichbare
  Facetten tief in der Tasche.
- `--stl <pfad>` — horizontaler Schnitt durch ein reales STL-Mesh
  (abhängigkeitsfreier Slicer), nutzt damit dieselben Beispiele wie die
  3D-Pipeline.

## Beispielergebnisse (Default-Config)

| Szene | erreichbar | Greedy (Posen) | ILP (Posen) |
|---|---|---|---|
| box | 64/64 | 19 | 10 |
| notched_box | 68/71 | 17 | 15 |
| slice TestKorper1 | 54/54 | 15 | 10 |

## Nicht enthalten / Abgrenzung

- Kein z/Pitch (2D). Die 4-DoF-Projektion ist hier eine 3-DoF-(x,y,yaw)-
  Projektion; die Pitch-Klemmung entfällt mangels dritter Dimension.
- Kein Tracking-/Standortmodell und keine Repositionierungs-Optimierung.
- Kollision nur als 1-NN-Clearance-Proxy; echte Freiraumprüfung erst im
  ROS2/Gazebo-Teil.
- Euklidische Distanzen in der Sequenzierung (keine Roadmap).
