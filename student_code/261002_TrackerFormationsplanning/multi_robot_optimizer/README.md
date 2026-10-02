# multi_robot_optimizer

## Overview

Das Paket plant Formationen mobiler Tracker-Roboter für drohnengestützte Messungen. Der Kartenknoten stellt die Belegungskarte und die Drohnenposen bereit. Der Optimierungsknoten berechnet daraus die Standorte der Roboter.

Das Verfahren arbeitet in zwei Ebenen. Die äußere Ebene (`optimizer_node.py`) teilt die Drohnenposen mit K-Means in Gruppen ein und erhöht die Gruppenanzahl schrittweise, bis sich die Gesamtkosten nicht mehr verbessern. Die innere Ebene (`pso_algorithm.py`) bestimmt für jede Gruppe mit einer Partikelschwarmoptimierung eine Formation. Als Gütemaß dient das A-Kriterium, also die Spur der inversen Fisher-Informationsmatrix (Cramér-Rao-Schranke). Das Modul `pso_algorithm.py` hängt nicht von ROS 2 ab.

**Author:** Christoph Verhage

**E-Mail:** christophver1999@gmail.com


## Usage

Ein Planungslauf benötigt zwei Knoten. Der Optimierungsknoten wartet auf Karte und Drohnenposen, der Kartenknoten sendet beides. Der Optimierungsknoten muss zuerst laufen.

**Terminal 1 – Optimierungsknoten:**

```bash
cd ~/map_ws && source install/setup.bash
ros2 run multi_robot_optimizer optimizer_node --ros-args \
    -p scenario_name:=1 -p timer_period:=0.5 \
    -p formation_export:=~/map_ws/loesung/finale.csv
```

**Terminal 2 – Kartenknoten (innerhalb von 60 Sekunden starten):**

```bash
cd ~/map_ws && source install/setup.bash
ros2 run multi_robot_optimizer image_to_ros --ros-args -p scenario:=1
```

Der Kartenknoten sendet Karte und Drohnenposen etwa zehn Sekunden lang und beendet sich. Der Optimierungsknoten untersucht danach nacheinander die Gruppenanzahlen K = 1, 2, 3 und beendet sich nach Erfüllung des Abbruchkriteriums. Die finale Lösung steht anschließend in `~/map_ws/loesung/finale.csv`.

Alternativ startet die Launch-Datei den Optimierungsknoten mit den Parametern aus der Konfigurationsdatei. Der Kartenknoten wird auch dann getrennt gestartet.

```bash
ros2 launch multi_robot_optimizer optimizer.launch.py
```

**Darstellung in RViz (optional):** RViz mit `rviz2` starten, als *Fixed Frame* `map` eintragen und die Anzeigen `MarkerArray` für die Topics `formation_markers` und `drone_targets_rviz` sowie `Marker` für `debug_grid` hinzufügen.

**Übergabe von Parametern:** Parameter werden beim Start mit `--ros-args -p name:=wert` übergeben. Gleitkommaparameter benötigen einen Dezimalpunkt (`1.0` statt `1`), Listen werden in eckigen Klammern angegeben. Unbekannte oder falsch geschriebene Parameternamen ignoriert ROS 2 ohne Meldung.


## Config files

- `pso_params.yaml`: Parameter des Optimierungsknotens (Gewichte, Flotte, Partikelschwarmoptimierung, Sicherheitsabstände, Wartezeit je Gruppenanzahl). Die Datei wird nur beim Start über die Launch-Datei geladen. Die Versuchsreihen übergeben ihre Parameter direkt über `run_batch.py`. Abweichend vom Standardwert des Knotens setzt die Datei `w_move: 1.6`.


## Launch files

- `optimizer.launch.py`: Startet den Optimierungsknoten `formation_optimizer_node` mit den Parametern aus `config/pso_params.yaml`. Die Launch-Datei besitzt keine Argumente.


## Nodes

### map_publisher_node

Ausführbare Datei: `image_to_ros`. Der Knoten erzeugt die Belegungskarte aus einem Luftbild oder einem synthetischen Szenario. Die Drohnenposen erzeugt er selbst oder lädt sie aus einer Datei. Er sendet beides etwa zehn Sekunden lang und beendet sich.

#### Published Topics

- `/map` (nav_msgs/OccupancyGrid): Belegungsraster mit 0,1 m je Zelle.
- `/drone_targets` (geometry_msgs/PoseArray): Drohnenposen (Position, Gierwinkel) im Rahmen `map`. Entfällt bei `drohnen_quelle:=extern`.

#### Parameters

| Parameter | Typ | Standard | Bedeutung |
|---|---|---|---|
| `scenario` | `int` | `0` | `0` = Luftbild aus `custom_image_path`; `1` bis `4` = synthetische Szenarien |
| `custom_image_path` | `string` | `/home/vboxuser/map_ws/Mein_Luftbild.png` | Luftbild für `scenario:=0`. Der Standardwert verweist auf den Entwicklungsrechner, der Pfad ist daher stets anzugeben. |
| `drohnen_quelle` | `string` | `generiert` | Herkunft der Drohnenposen: `generiert`, `datei` oder `extern` |
| `drohnen_datei` | `string` | leer | CSV-Datei mit Drohnenposen für `drohnen_quelle:=datei` |
| `drohnen_export` | `string` | leer | Speichert die verwendeten Drohnenposen als CSV (Vorlage oder Kontrolle) |

Die synthetischen Szenarien bilden jeweils eine Fläche von 20 m × 20 m ab:

| Szenario | Messobjekt | Weitere Hindernisse | Drohnenposen |
|---|---|---|---|
| 1 | Quadrat 4 m × 4 m | keine | 24 |
| 2 | Quadrat 4 m × 4 m | Wand | 24 |
| 3 | U-Profil 10 m × 8 m | keine | 76 |
| 4 | U-Profil 10 m × 8 m | Wand, zwei diagonale Riegel | 72 |

### formation_optimizer_node

Ausführbare Datei: `optimizer_node`. Der Knoten berechnet das Distanzfeld der Karte, plant die Formationen, stellt sie dar, protokolliert die Kennzahlen und exportiert die finale Lösung.

#### Subscribed Topics

- `/map` (nav_msgs/OccupancyGrid): Belegungsraster der Umgebung.
- `/drone_targets` (geometry_msgs/PoseArray): Drohnenposen im Rahmen `map`.

Der Knoten übernimmt jeweils die erste empfangene Karte und die ersten empfangenen Drohnenposen. Da beide Knoten Standardnachrichten verwenden, lassen sich Karte oder Drohnenposen auch aus anderen Quellen einspeisen.

#### Published Topics

- `formation_markers` (visualization_msgs/MarkerArray): Formationen und Gruppen.
- `drone_targets_rviz` (visualization_msgs/MarkerArray): Drohnenposen, eingefärbt nach Gruppe.
- `debug_grid` (visualization_msgs/Marker): Hindernisse und Distanzfeld zur Kontrolle.

#### Parameters

| Parameter | Typ | Standard | Bedeutung |
|---|---|---|---|
| `w_move` | `float` | `1.0` | Gewicht der Bewegungskosten (µm²/m) |
| `w_obs` | `float` | `1.0` | Gewicht der Hinderniskosten |
| `robot_types` | `string[]` | `['A','A','A','A','B','B']` | Typ je Roboter: A = ein Tracker, B = zwei Tracker im Abstand von 0,2 m |
| `start_robot_poses` | `float[]` | 6 Roboter im Raster | Ausgangsaufstellung `[x1, y1, x2, y2, …]` in m; legt die Roboteranzahl fest |
| `d_safe` | `float` | `1.0` | Wirkungsbereich der weichen Hindernisstrafe in m |
| `d_crash` | `float` | `0.15` | Kollisionsradius gegenüber Hindernissen in m |
| `max_iterations` | `int` | `50` | Höchstzahl der PSO-Iterationen je Gruppe |
| `t_patience` | `int` | `15` | Iterationen ohne Verbesserung bis zum Abbruch |
| `epsilon` | `float` | `0.0001` | Mindestverbesserung der besten Position |
| `inertia_weight` | `float` | `0.5` | Trägheitsgewicht der PSO |
| `c1`, `c2` | `float` | `1.5` | Beschleunigungskoeffizienten der PSO |
| `timer_period` | `float` | `4.0` | Sekunden je untersuchter Gruppenanzahl (nur Wartezeit, ohne Einfluss auf das Ergebnis) |
| `startup_timeout` | `float` | `60.0` | Sekunden bis zum Selbstabbruch, falls keine Karte oder Drohnenposen eintreffen |
| `csv_path` | `string` | `~/map_ws/pso_evaluation_results.csv` | Ergebnisdatei mit den Kennzahlen je Lauf (wird fortgeschrieben) |
| `formation_export` | `string` | leer | Pfad für die finale Lösung; leer = kein Export |
| `task_id` | `string` | leer | Kennung des Laufs in der Ergebnisdatei (muss mit einem Buchstaben beginnen) |
| `scenario_name` | `int` | `1` | Szenarionummer für die Bezeichnung in der Ergebnisdatei |
| `batch_mode` | `bool` | `true` | `true`: Der Knoten schreibt die Ergebnisdatei und beendet sich. `false` (Vorführung): Der Knoten bleibt nach der Planung aktiv, sendet die gewählte Formation weiter an RViz und speichert den Verlauf der Gesamtkosten als Diagramm. Der Speicherort ist in `plot_results()` fest eingetragen (`/home/vboxuser/map_ws/j_total_plot.png`) und muss angepasst werden. |

Fehlen Einträge in `robot_types`, ergänzt der Knoten Roboter vom Typ A. Die Flotte muss mindestens sechs Tracker umfassen.


## Eingabe: Drohnenposen

Standardmäßig erzeugt der Kartenknoten die Drohnenposen selbst: Messsäulen im Abstand von etwa 2,5 m zum Messobjekt mit je vier Höhen (1,25 / 3,75 / 6,25 / 8,75 m). Stammen die Posen aus einer anderen Anwendung, etwa einer Flugplanung, gibt es zwei Wege.

**Aus einer Datei (`drohnen_quelle:=datei`):**

```bash
ros2 run multi_robot_optimizer image_to_ros --ros-args \
    -p scenario:=1 -p drohnen_quelle:=datei -p drohnen_datei:=~/map_ws/drohnenposen.csv
```

Die Datei enthält eine Pose je Zeile im Format `x, y, z[, yaw]`:

```
x,y,z,yaw
3.3850,-3.5550,1.2500,2.3317
3.3850,-3.5550,3.7500,2.3317
6.0,6.0,4.0
```

Die Koordinaten liegen im Kartenrahmen `map` in Metern: Ursprung in der Kartenmitte, x nach rechts, y nach oben. Der Gierwinkel in Radiant ist optional. Fehlt er, richtet sich die Drohne zum Messobjekt aus. Trennzeichen ist das Komma (alternativ Semikolon), Dezimalzeichen der Punkt. Eine Kopfzeile und Kommentare mit `#` sind erlaubt. Die Datei sollte mit einem Texteditor bearbeitet werden. Tabellenprogramme mit deutscher Spracheinstellung speichern Dezimalkommas, die nicht gelesen werden können.

Beim Laden verwirft der Knoten Posen außerhalb der Karte, in Hindernissen oder mit z ≤ 0 und meldet jede einzeln. Die Zeile `--> X von Y Drohnenposen … übernommen` zeigt das Ergebnis. Eine Vorlage erzeugt der Parameter `drohnen_export`:

```bash
ros2 run multi_robot_optimizer image_to_ros --ros-args -p scenario:=1 -p drohnen_export:=~/map_ws/drohnenposen.csv
```

**Von einem anderen ROS-2-Knoten (`drohnen_quelle:=extern`):** Der Kartenknoten sendet dann nur die Karte. Ein anderer Knoten sendet die Posen als `geometry_msgs/PoseArray` im Rahmen `map` auf `/drone_targets`. Er sollte die Nachricht mehrfach senden, da der Optimierungsknoten nur Nachrichten empfängt, die nach seinem Start eintreffen.


## Ausgabe: Ergebnisse und finale Lösung

**Ergebnisdatei (`csv_path`):** Jeder Lauf hängt eine Zeile an. Die wichtigsten Spalten:

| Spalte | Inhalt |
|---|---|
| `Task_ID`, `Messobjekt`, `Status` | Kennung, Szenario, Ende des Laufs (`abbruchkriterium`, `max_k`, `keine_loesung`) |
| `Gewaehltes_k`, `Finale_Kosten_J` | gewählte Gruppenanzahl K* und Gesamtkosten |
| `cost_crlb`, `cost_move`, `cost_obs`, `cost_inter` | Kostenanteile (Schätzgüte in µm², gewichtete Bewegungs- und Hinderniskosten, Roboterabstand) |
| `k_Verlauf`, `Kosten_Verlauf` | untersuchte zulässige Gruppenanzahlen und ihre Gesamtkosten |
| `Rechenzeit_Verlauf_s`, `Rechenzeit_gesamt_s` | Rechenzeit je untersuchter Gruppenanzahl und gesamt |
| `w_move`, `w_obs`, `d_safe`, `d_crash`, `robot_types`, `N_Roboter`, `N_Drohnenposen` | Konfiguration des Laufs |
| `Gruppengroessen`, `Formationen` | Posen je Gruppe und Roboterpositionen je Formation (JSON) |

**Finale Lösung (`formation_export`):** Ist ein Pfad gesetzt, schreibt der Knoten am Ende zwei Dateien. Eine Kommentarzeile nennt jeweils Szenario, K, Gesamtkosten und Gewichte.

`finale.csv` – Roboterpositionen je Formation:

```
gruppe,roboter,typ,x,y,start_x,start_y
1,1,A,-0.6952,-5.8010,0.0000,0.0000
```

`finale_drohnen.csv` – welche Drohnenposen jede Formation erfasst:

```
gruppe,x,y,z,yaw
1,3.3850,-3.5550,1.2500,2.3317
```

Endet ein Lauf ohne zulässige Lösung, entsteht keine Exportdatei.


## Feste Modellwerte im Quelltext

Einige Werte sind keine ROS-Parameter, sondern stehen als Konstanten im Quelltext:

| Wert | Datei | Bedeutung |
|---|---|---|
| σ_pos = 0,2 µm + 0,3 µm/m · d | `pso_algorithm.py` | Abstandsunsicherheit der Tracker |
| d_max = 30 m | `pso_algorithm.py` | Messreichweite |
| σ_ang = 0,05 rad + 0,01 rad/m · d | `pso_algorithm.py` | angenommene Winkelunsicherheit |
| d_min = 1,5 m, Kollision bei 0,6 m | `pso_algorithm.py` | Abstände zwischen Robotern |
| Strafwerte 10⁸ / 10⁷ / 10⁹ / 10⁶ | `pso_algorithm.py` | nicht beobachtbare Pose / je ungültigem Tracker / numerischer Fehler / Kollision |
| v_max = 2 m, Startbereich ± 15 m | `pso_algorithm.py` | Geschwindigkeitsgrenze und Initialisierung der PSO |
| Populationsgröße max(30, 10 · 2N) | `pso_algorithm.py` | Anzahl der Partikel |
| 0,1 m je Zelle, Arbeitsabstand 2,5 m | `image_to_ros.py` | Kartenauflösung und Abstand der Drohnen zum Messobjekt |
| K-Means mit festem Startwert, 10 Wiederholungen | `optimizer_node.py` | reproduzierbare Gruppierung |

Die Positionsvarianzen und das A-Kriterium haben die Einheit µm².
