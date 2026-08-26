# vpp3d — 3D-Portierung des alternativen View-Planning-Ansatzes

Eigenständige, **vom restlichen Code getrennte** 3D-Portierung des 2D-Prototyps
[`../vpp2d`](../vpp2d/README.md). Dieselbe Pipeline und Constraint-Struktur,
aber im vollen Raum: die 4-DoF-Kinematik (x, y, z, yaw) ist hier vollständig
ausgeprägt.

> **Fokus: reine Sichtlinie (line of sight)** für die Drohnen-Sensorabdeckung.
> Das innere Set Cover ist single-level über die Sichtbarkeitsmatrix. Ergänzt um
> die **Registrierungs-Überlappung** ([5], `overlap.py` / `setcover.py`) — als
> Konnektivität des flächenbasierten Registrierungsgraphen, entweder nachgelagert
> repariert oder gemeinsam mit der Abdeckung als **Connected Set Cover** exakt
> gelöst — und ein **LoS-Tracking-Modell** für die bodengebundenen mobilen
> Tracker (`tracking.py`).

## Was gegenüber 2D hinzukommt

| Aspekt | 2D (vpp2d) | 3D (vpp3d) |
|---|---|---|
| Primitiv | Polygon-Kantensegmente | **Mesh-Dreiecke** (zur Auflösung midpoint-unterteilt) |
| Pose | (x, y, yaw) | (x, y, z, yaw) — **Pitch geklemmt** auf den Gimbal-Bereich, Roll gesperrt |
| Sichtfeld | Kreissegment | **Sehpyramide** (horizontal × vertikal) |
| Kegel-Sampling | Inzidenzwinkel in der Ebene | Frontalstrahl + **Azimut-Ringe** um die Normale |
| Dedup-Schlüssel | (Voxel x,y) × Yaw-Bin | (Voxel x,y,z) × Yaw-Bin × **Pitch-Bin** |
| Raycast | analytisch (Kanten) | **Open3D** `RaycastingScene` gegen das Mesh |
| Visualisierung | matplotlib 2D | matplotlib-3D-PNG **+ Open3D-Interaktiv** |

Der **Machbarkeitsfilter** nach der Pitch-Klemmung verwirft Posen, bei denen die
Facette aus dem vertikalen Sichtfeld fällt — dadurch sind *horizontale* Flächen
(z. B. die Würfeloberseite) physikalisch unabdeckbar (Strahlen müssten zu steil
geneigt sein). Das ist bewusst akzeptiert: Vollständigkeit gilt relativ zur
*erreichbaren* Fläche.

## Pipeline (Spiegel des Bild-Entwurfs, in 3D)

| Schritt | Modul | Inhalt |
|---|---|---|
| [1] Facetten als Primitiv | `scene.py` | Mesh-Dreiecke → Facetten (Mittelpunkt, Außennormale, Fläche) |
| [2] Posenregion je Facette samplen | `candidates.sample_pose_regions` | Kegel-Sampling um die Normale × Distanzschale |
| [3] Projektion auf 4-DoF-Raum + Dedup | `candidates.project_and_dedup` | Yaw exakt, Pitch geklemmt; Voxel/Yaw/Pitch-Dedup |
| [4] Coverage-Matrix per Raycast, duale Sicht | `visibility.py` | Projektor + Kamera, Open3D-Raycast |
| [5] Registrierungs-Überlappung | `overlap.py` | flächenbasierter Registrierungsgraph + Konnektivitäts-Reparatur (`ensure_connected`) |
| [5b] Lokale Pose-Verfeinerung | `refine.py` | gewählte Posen per Nelder-Mead auf ein glattes Qualitätsoptimum schieben (optional, `--refine`) |
| [6] Set Cover (Greedy + ILP + Connected) | `setcover.py` | Greedy/ILP (rein matrixbasiert) + `connected_set_cover_ilp` (Abdeckung + Zusammenhang gemeinsam) |
| Tracking-Standorte (LoS) | `tracking.py` | bodengebundene Tracker, Set Cover mit Sichtlinien-Raycast |
| [7] TSP-Tour | `sequencing.py` | zweistufig: TSP über Tracking-Standorte + offener TSP je Segment (NN + 2-opt, 3D-Distanzen) |

### [5] Registrierungs-Überlappung (`overlap.py` / `setcover.py`)

Set Cover garantiert per Default (`--k 1`) nur, dass jede erreichbare Facette von
*mindestens einem* Scan gesehen wird — nicht, dass sich die Scans zu *einem*
gemeinsamen Koordinatensystem registrieren lassen. Die ICP-Registrierung braucht,
dass benachbarte Scans gemeinsame Fläche teilen.

Modelliert als **Registrierungsgraph**: Knoten = gewählte Scans, Kante (a, b),
wenn beide Scans mindestens `min_overlap` Anteil gemeinsam gesehener Facetten
teilen — **bezogen auf den kleineren der beiden Scans**. Bei uniformer
Diskretisierung ist die Facettenzahl proportional zur Fläche, d. h. `min_overlap`
ist ein echter **Flächen-Anteil**: `0.25` heißt „25 % der Fläche des kleineren
Scans werden auch vom Nachbarn gesehen". (Der frühere Ansatz maß das *äußere
Rand-Band* im Bild-Radius — 25 % Radius ≈ 44 % Fläche — und erzwang viele
unnötige Brücken-Posen.)

Gefordert ist der **Zusammenhang** des Graphen (jeder Scan über eine Kette von
Paar-Registrierungen ins Weltsystem verkettbar). Zwei Wege:

* **Zweistufig** — `overlap.ensure_connected` ergänzt nach dem Set Cover greedy
  Brücken-Posen, bis der Graph zusammenhängt. Läuft fair nach Greedy *und* ILP
  (`--method greedy|ilp`).
* **Gemeinsam (exakt)** — `setcover.connected_set_cover_ilp` (`--method
  connected`) optimiert Abdeckung und Zusammenhang in *einem* CP-SAT-Modell
  (Single-Commodity-Flow über die Overlap-Kanten). Näher am echten Optimum, weil
  die Basisauswahl die Registrierbarkeit schon berücksichtigt statt sie
  nachträglich zu reparieren; als exakte Referenz für kleine/mittlere Instanzen
  gedacht.

In der Visualisierung sind die Facetten, die **≥ 2 Scans** sehen (die tatsächliche
Registrierungs-Überlappung), **orange** hervorgehoben.

### [5b] Lokale Pose-Verfeinerung (`refine.py`)

Die Kandidatengenerierung [2/3] erzeugt Posen auf einem **diskreten Raster**
(feste Distanz-Stützstellen × Azimut-Ringe). Die vom Set Cover gewählten Posen
sitzen daher selten im Optimum ihres Footprints — oft etwas zu nah/fern, mit
schrägem Einfall oder am Rand des Sichtfelds. `refine.py` verschiebt jede
gewählte Pose einzeln per **Nelder-Mead** auf ein lokales Optimum einer glatten
Qualitäts-Score

    J = q_dist · q_incidence · q_frustum

(Distanz-Glocke um `d_opt` · Einfalls-Smoothstep bis `θ_max` · Frustum-
Zentrierung). Die Orientierung ist kein freier Parameter, sondern wird je Schritt
auf den qualitätsgewichteten Facetten-Schwerpunkt ausgerichtet (Yaw exakt, Pitch
geklemmt, Roll = 0 — 4-DoF wie [2/3]).

**Abdeckungsbewusst (Default):** Jede Pose darf sich nur so weit verschieben,
dass sie ihre vom Set Cover zugewiesenen Facetten (Zeile von V) weiterhin sieht —
eine Barriere bestraft jede zugewiesene Facette, die unter die Sichtbarkeit
rutscht. Ohne diese Einschränkung kollabieren benachbarte Posen aufeinander und
die Abdeckung bricht ein (Ablation `--refine-free`: `notched_box` fällt von
68,8 % auf 53,8 % erreichbarer Facetten).

**Abdeckungsmaximierend (`--gain-weight g`):** Zusätzlich zu den zugewiesenen
Facetten werden benachbarte, noch erreichbare Facetten mit Gewicht `g` in die
Score aufgenommen — die Pose nimmt so weitere Facetten mit. Die Distanz-Glocke
`q_dist` verhindert dabei, dass sie zum Einsammeln entfernter Facetten auf
`d_max` driftet (dort → `q_dist = 0`, und die Barriere der zugewiesenen Facetten
schlägt zu). Auf `box`: weiche Abdeckung +8,9 % Facetten/Pose bei weiter
sinkendem `|d − d_opt|` (−36 %), also **kein** d_max-Drift.

Die Score ist rein geometrisch (kein Raycast); nach der Verschiebung wird die
Sichtbarkeit [4] neu berechnet (maßgeblicher Verdeckungs-Check) — `run.py` gibt
die neu geraycastete Abdeckung vorher/nachher aus. Beispiel `box`
(abdeckungsbewusst): `|d − d_opt|` −66 %, Einfallswinkel 16,6° → 13,9°, Abdeckung
66,7 % → 66,4 % (nahezu erhalten).

### Tracking mit Sichtlinie (`tracking.py`)

Die bodengebundenen, mobilen Tracking-Roboter brauchen **freie Sichtlinie** zur
Drohne. Kandidaten-Standorte liegen auf einem Bodenraster über der Posen-Hülle;
ein Standort erfasst eine Pose, wenn die Drohne im Arbeitsabstand
`[range_min, range_max]` liegt **und** der Strahl Tracker → Drohne unverdeckt am
Messobjekt vorbeigeht (Open3D-Raycast). Minimale Standortmenge = Greedy-Set-Cover
über die Posen; jede Pose wird dem nächstgelegenen erfassenden Standort
zugeordnet (blaue Marker + Sichtlinien in der Visualisierung).

## Aufruf

Aus dem Verzeichnis `Code/` (venv mit numpy/scipy/open3d/ortools/matplotlib):

```powershell
.venv\Scripts\python -m vpp3d.run                                       # box, Greedy
.venv\Scripts\python -m vpp3d.run --scene notched_box --method both     # Greedy vs. ILP
.venv\Scripts\python -m vpp3d.run --mesh Meshes/TestKorper1.stl --method both
.venv\Scripts\python -m vpp3d.run --mesh Meshes/EasyCube.ply --assume-convex --resolution 0.6  # Punktwolke -> Poisson
.venv\Scripts\python -m vpp3d.run --scene notched_box --baseline 0      # Ablation: Einzelsicht
.venv\Scripts\python -m vpp3d.run --mesh Meshes/TestKorper1.stl --show  # + Open3D-Ansicht
.venv\Scripts\python -m vpp3d.run --scene box --refine                  # [5b] gewählte Posen lokal verfeinern
.venv\Scripts\python -m vpp3d.run --scene box --refine --gain-weight 0.3 # [5b] + Abdeckung maximieren (kein d_max-Drift)
.venv\Scripts\python -m vpp3d.run --scene notched_box --refine --refine-free  # Ablation: Abdeckung bricht ein
```

Verfeinerungs-Optionen (`--refine`): `--refine-free` (Ablation ohne
Facetten-Zuweisung), `--gain-weight g` (Abdeckungsmaximierung, Default 0 = aus),
`--max-offset` (Suchradius je Pose, Default 0,5·d_opt), `--sigma-scale` (Breite
der Distanz-Glocke), `--coverage-weight` (Abdeckungs-Barriere, Default 4,0),
`--z-min` (Bodenfilter).

Statische Ergebnis-PNGs (Kandidatenposen + Lösung mit Facettenstatus, gewählten
Posen und Route) landen in `vpp3d/output/`. `--show` öffnet zusätzlich die
interaktive Open3D-Ansicht.

### Schritt-für-Schritt-Viewer (Greedy)

`stepviz.py` zeigt den Aufbau der Abdeckung Pick für Pick (Pendant zu
`../vpp2d/stepviz.py`): je Schritt die gewählte Pose mit Blickrichtung, ihre
Sichtstrahlen und die **neu** (orange) vs. **überlappend** (blau) erfassten
Facetten; bereits abgedeckt = grün, offen = rot, unerreichbar = grau.

```powershell
.venv\Scripts\python -m vpp3d.stepviz --scene notched_box           # PNG-Frames je Schritt
.venv\Scripts\python -m vpp3d.stepviz --scene notched_box --show    # interaktiver Navigator
.venv\Scripts\python -m vpp3d.stepviz --mesh Meshes/TestKorper1.stl
```

Im `--save`-Modus (Default) landen die Frames in
`vpp3d/output/steps_<szene>_greedy/step_000.png …`. Im `--show`-Modus blättern
`→`/`n` und `←`/`b` durch die Schritte (Home/End = Anfang/Ende, `s` = Frame
speichern); die Kameraperspektive bleibt beim Blättern erhalten.

### Interaktiver Inspektor (`inspector.py`)

`inspector.py` ist der Haupt-Debug-Viewer für die fertige Lösung. Er führt die
komplette Pipeline ([2]–[7]) intern aus und zeigt das Meshobjekt mit korrekt
**gefärbten Facetten-Dreiecken** (kein Punktraster — echte Mesh-Flächen).

```powershell
# Synthetische Szene
.venv\Scripts\python -m vpp3d.inspector
.venv\Scripts\python -m vpp3d.inspector --scene notched_box

# Reales Mesh / Punktwolke
.venv\Scripts\python -m vpp3d.inspector --mesh Meshes/TestKorper1.stl
.venv\Scripts\python -m vpp3d.inspector --mesh Meshes/TestKorper1.stl --method both
.venv\Scripts\python -m vpp3d.inspector --mesh Meshes/EasyCube.ply --assume-convex

# ILP statt Greedy; Connected Set Cover (Abdeckung + Zusammenhang gemeinsam)
.venv\Scripts\python -m vpp3d.inspector --method ilp --time-limit 120
.venv\Scripts\python -m vpp3d.inspector --method connected --time-limit 120
# Konnektivitäts-Reparatur deaktivieren
.venv\Scripts\python -m vpp3d.inspector --no-overlap

# [5b] Lokale Pose-Verfeinerung (Sichtbarkeit wird danach neu geraycastet)
.venv\Scripts\python -m vpp3d.inspector --refine
.venv\Scripts\python -m vpp3d.inspector --refine --gain-weight 0.3
```

#### CLI-Optionen

| Option | Standard | Beschreibung |
|---|---|---|
| `--scene` | `box` | Synthetische Testszene: `box`, `notched_box` |
| `--mesh` | — | Reales Mesh oder Punktwolke (STL/PLY); ersetzt `--scene` |
| `--assume-convex` | aus | Normalen radial nach außen orientieren (für Punktwolken ohne Mesh-Normalen) |
| `--resolution` | Config | Facetten-Auflösung in m überschreiben — kleinere Werte → mehr Facetten → langsamere Berechnung |
| `--method` | `greedy` | Set-Cover-Solver: `greedy`, `ilp` oder `connected` (Connected Set Cover: Abdeckung + Zusammenhang gemeinsam) |
| `--k` | `1` | k-Coverage: jede Facette von ≥ k Posen sehen lassen (Redundanz für die Registrierung); Bedarf auf erreichbare Abdeckung gekappt |
| `--time-limit` | `60` | ILP-Zeitlimit in Sekunden |
| `--baseline` | Config | Clearance-Baseline in m; `0` = Einzelsicht-Ablation |
| `--no-overlap` | aus | Konnektivitäts-Reparatur (Schritt [5]) überspringen |
| `--refine` | aus | [5b] gewählte Posen lokal verfeinern; danach wird die Sichtbarkeit neu geraycastet (Einfärbung zeigt die verfeinerten Posen) |
| `--refine-free` | aus | Ablation zu `--refine`: ohne Facetten-Zuweisung (Abdeckung bricht ein) |
| `--gain-weight` | `0` | `--refine`: > 0 nimmt benachbarte Facetten mit Gewicht `g` in die Score auf (Abdeckungsmaximierung, kein d_max-Drift) |
| `--max-offset` | `0.5·d_opt` | `--refine`: max. Suchradius je Pose in m |
| `--sigma-scale` | `1.5` | `--refine`: Breite der Distanz-Glocke |
| `--coverage-weight` | `4.0` | `--refine`: Abdeckungs-Barriere je zugewiesener Facette (0 = aus) |
| `--z-min` | — | `--refine`: Bodenfilter, Posen nicht unter z_min schieben |
| `--config` | `config.toml` | Alternative Konfigurationsdatei |

#### Tastenkürzel

| Taste | Funktion |
|---|---|
| `O` / `3` | **Übersicht** — alle Plan-Posen, Gesamtabdeckungsstatus |
| `M` / `4` | **Mosaik** — jede Pose hat eine Farbe, ihr Footprint entsprechend; Registrierungs-Überlappung (≥2 Scans) gelb; Kandidaten ausgeblendet, Kamera-Pyramiden sichtbar |
| `C` / `1` | **Kandidaten** — alle Kandidatenposen, eine nach der anderen durchsteppen |
| `P` / `2` | **Plan** — nur gewählte Posen, mit FoV-Frustum der aktiven Pose |
| `N` / `→` | Nächste Pose (Kandidaten- und Plan-Modus) |
| `B` / `←` | Vorherige Pose |
| `S` | Screenshot → `vpp3d/output/inspector_<szene>_<modus>_<nr>_<ts>.png` |
| `Q` / `Esc` | Viewer schließen |

#### Facetten-Einfärbung je Modus

| Farbe | Kandidaten | Plan | Übersicht | Mosaik |
|---|---|---|---|---|
| Hellgrün | Von akt. Pose sichtbar | Von akt. Pose sichtbar | Abgedeckt (Plan) | Farbe der zuständigen Plan-Pose |
| Blassgrün | — | Abgedeckt (andere Plan-Posen) | — | — |
| Rot | — | Coverable, nicht abgedeckt | Coverable, nicht abgedeckt | Coverable, nicht abgedeckt |
| Hellgrau | Coverable, nicht sichtbar | — | — | — |
| Grau | Unerreichbar | Unerreichbar | Unerreichbar | Unerreichbar |
| Gelb | — | — | — | Registrierungs-Überlappung (von ≥ 2 Scans gesehen) |

Im **Mosaik-Modus** werden zusätzlich kleine **Kamera-Pyramiden** für jede Plan-Pose
eingeblendet — in der Palettenfarbe der jeweiligen Pose, mit der tatsächlichen
FoV-Öffnung skaliert.

---

### Laufzeit-Logging (`runlog.py`)

Jeder Aufruf von `run.py` und `inspector.py` legt automatisch einen Ordner
`vpp3d/output/runs/<YYYY-MM-DD_HH-MM-SS>_<szene>/` an. Darin werden gespeichert:

| Datei | Inhalt |
|---|---|
| `visibility_matrix.png` | Sichtbarkeitsmatrix der gewählten Posen × Facetten als Heatmap; Spalten nach Abdeckungsdichte sortiert; bei sehr großen Matrizen heruntergesampelt (max. 2000 × 800 px) |
| `<szene>_<methode>.png` | Kopie der Übersichts-PNG (nur `run.py`) |

---

### Testszenen

- `box` — konvexer 4-m-Würfel, 3D-Analogon zu **EasyCube** (alle Seitenflächen
  frei einsehbar; Ober-/Unterseite horizontal → unerreichbar).
- `notched_box` — Quader (4,4 × 2 × 3,1 m) mit rechteckiger Tasche, 3D-Analogon
  zu **TestKorper1**: erzeugt Selbstverdeckung (die duale Sicht verwirft Posen,
  bei denen die Kamera durch die Taschenkante verdeckt wird) und 90°-Kanten.
- `--mesh <pfad>` — reales Mesh (STL/PLY). Punktwolken (z. B. `EasyCube.ply`)
  werden per Poisson-Rekonstruktion zu einem Occluder-Mesh; `--assume-convex`
  orientiert die Normalen radial nach außen. Für große Objekte (EasyCube.ply ist
  ~11 × 11 × 6 m) die Facetten-Auflösung gröber wählen (`--resolution 0.6`),
  sonst explodiert die Facetten- und Posenzahl.

## Beispielergebnisse (Default-Config, duale Sicht, baseline = 0,4 m)

| Szene | erreichbar | Greedy | ILP | Faktor |
|---|---|---|---|---|
| box | 8192/12288 (66,7 %) | 44 | 36 | 1,22 |
| notched_box | 16312/28672 (56,9 %) | 64 | 53 | 1,21 |
| TestKorper1.stl | 5120/7168 (71,4 %) | 39 | 29 | 1,34 |

Zum Vergleich liefert die große Pipeline für TestKorper1 Greedy 37 / ILP 30 —
der alternative Ansatz reproduziert die Größenordnung. Die Einzelsicht-Ablation
(`--baseline 0`) braucht für `notched_box` 62 statt 64 Scans, weil die
Kamera-Verdeckungsprüfung an der Tasche entfällt.

## Nicht enthalten / Abgrenzung

- Kein bi-level Set Cover: Tracking-Standorte werden *nach* der Posenwahl
  bestimmt (nachgelagert), nicht gemeinsam mit ihr optimiert. Die
  *Sequenzierung* koppelt beide jedoch bereits zweistufig (äußere Tour über die
  Standorte, innere offene TSP je Segment), sodass die Drohne nicht mehr
  zwischen Tracking-Bereichen hin- und herspringt.
- Kollision nur als 1-NN-Clearance-Proxy; echte Freiraumprüfung (ESDF/OctoMap)
  erst im ROS2/Gazebo-Teil.
- Euklidische Distanzen in der Sequenzierung (keine kollisionsfreie Roadmap).
- Occluder ist das grobe Eingangs-Mesh; nur die Facetten werden zur Auflösung
  unterteilt (kein Crack/T-Stoß, da uniform unterteilt).
