# UAV View Planning für Streifenlicht-Messsysteme

## Overview

Code zur Studienarbeit „Entwicklung eines Algorithmus zur optimalen Abtastung von
großskaligen Messobjekten mittels UAV-basierter Sensorik".

Eine Drohne mit Streifenlicht-Projektionssystem vermisst großskalige Bauteile.
Der Algorithmus berechnet offline eine minimale, vollständige und registrierbare
Menge von Sensorposen — Set Cover über eine geraycastete Sichtbarkeitsmatrix, mit
4-DoF-Kinematik (x, y, z, yaw), Sensor-Constraints (Arbeitsabstand,
Einfallswinkel, Sichtfeld, duale Sicht Projektor/Kamera) und bodengebundenen
Tracking-Einheiten. Reines Python, kein ROS.

* `vpp2d` — 2D-Prototyp, gleiche Constraints, läuft in Sekunden
* `vpp3d` — volle 3D-Implementierung inkl. Registrierungsgraph, Pose-Verfeinerung,
  Tracking-Standorten und TSP-Sequenzierung

## Installation

**Python 3.12** (Open3D hat Stand 2026 keine Wheels für 3.13/3.14).

```bash
cd 260722_UAVViewPlanning
python3.12 -m venv .venv
.venv/bin/pip install -r requirements.txt      # Windows: .venv\Scripts\pip
```

Aufrufe immer aus **diesem** Verzeichnis als Modul, sonst stimmen die Pfade zu
`Meshes/` und `config.toml` nicht.

## Packages

### vpp2d

2D-Prototyp: Facetten → Posenregionen → Projektion/Dedup → Coverage-Matrix →
Set Cover → TSP. Details: [`vpp2d/README.md`](vpp2d/README.md).

```bash
python -m vpp2d.run                                   # Testszene box, Greedy
python -m vpp2d.run --scene notched_box --method both # Greedy vs. ILP
python -m vpp2d.stepviz --scene notched_box --show    # Schritt für Schritt
```

### vpp3d

Dieselbe Pipeline in 3D: Kegel-Sampling des Posenraums, Pitch-Klemmung,
Sichtbarkeit per Open3D-Raycast (duale Sicht), Set Cover mit Greedy / ILP /
Connected (OR-Tools CP-SAT), Registrierungsgraph, lokale Pose-Verfeinerung
(Nelder-Mead), Tracking-Standorte und zweistufiger TSP.
Details: [`vpp3d/README.md`](vpp3d/README.md), Kurzreferenz [`vpp3d/usage.md`](vpp3d/usage.md).

```bash
python -m vpp3d.run                                     # Testszene box, Greedy
python -m vpp3d.run --scene notched_box --method both   # Greedy vs. ILP
python -m vpp3d.run --mesh Meshes/TestKorper1.stl --method both
python -m vpp3d.run --scene box --refine                # + Pose-Verfeinerung
python -m vpp3d.inspector --mesh Meshes/TestKorper1.stl # interaktiver Viewer
python -m vpp3d.stepviz --scene notched_box --show      # Schritt für Schritt
```

Ergebnis-PNGs und Laufprotokolle landen in `vpp2d/output/` bzw. `vpp3d/output/`.
Alle Sensor- und Tracking-Parameter stehen in den jeweiligen `config.toml`.

## Scripts

Keine losen Skripte — alle Einstiegspunkte sind Module:

| Modul | Funktion |
|---|---|
| `vpp3d.run` | kompletter Durchlauf, Ergebnis-PNGs |
| `vpp3d.inspector` | interaktiver Open3D-Viewer der Lösung |
| `vpp3d.stepviz` | Greedy-Abdeckung Pick für Pick |
| `vpp3d.render_figures`, `vpp3d.stepfigs` | Abbildungen für die schriftliche Arbeit |
| `vpp2d.run`, `vpp2d.stepviz` | Pendants in 2D |

`Meshes/` enthält die Testkörper (`TestKorper1.stl` ≈ 4,4 × 2 × 3,1 m,
Punktwolke `EasyCube.ply`); `box` und `notched_box` werden zur Laufzeit erzeugt.
