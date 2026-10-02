# Formationsplanung mobiler Tracker-Roboter für drohnengestützte Messungen

## Overview

Dieser Ordner enthält die Implementierung zur Masterarbeit „Entwicklung eines Algorithmus zur Positionsberechnung für ein Multi-Roboter-System auf Basis von Drohnen-Messpunkten“.

Das Verfahren plant, wo eine Flotte mobiler Bodenroboter mit Trackern stehen muss, damit sie eine Drohne an allen vorgegebenen Messposen um ein Messobjekt erfassen kann. Es wägt dabei die Schätzgüte der Drohnenposen, Sicherheitsabstände zu Hindernissen und zwischen den Robotern sowie den Aufwand für Umpositionierungen gegeneinander ab.

Das Verfahren arbeitet in zwei Ebenen. Die äußere Ebene teilt die Drohnenposen mit K-Means in Gruppen ein und erhöht die Gruppenanzahl schrittweise, bis sich die Gesamtkosten nicht mehr verbessern. Die innere Ebene bestimmt für jede Gruppe mit einer Partikelschwarmoptimierung eine Formation der Roboter. Als Gütemaß dient das A-Kriterium, also die Spur der inversen Fisher-Informationsmatrix (Cramér-Rao-Schranke).

Die Umsetzung erfolgt in ROS 2 Jazzy. Die Ergebnisse lassen sich in RViz darstellen.

Inhalt des Ordners:

| Pfad | Inhalt |
|---|---|
| `multi_robot_optimizer/` | ROS-2-Paket mit Kartenknoten, Optimierungsknoten und Partikelschwarmoptimierung |
| `run_batch.py` | Automatisierte Versuchsreihen (parallel, wiederaufnehmbar) |
| `auswertung.py` | Statistische Auswertung der Versuchsreihen |
| `abbildungen_erstellen.py` | Erzeugung der Abbildungen der Arbeit |
| `ergebnisse/*_gesamt.csv` | Ergebnisse der Versuchsreihen der Arbeit |

**Author:** Christoph Verhage

**E-Mail:** christophver1999@gmail.com


## Installation

Voraussetzungen:

| Komponente | Version |
|---|---|
| Betriebssystem | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Python | 3.12 |
| Python-Bibliotheken | NumPy, SciPy, scikit-learn, scikit-image, OpenCV, Matplotlib |

Die Bibliotheken lassen sich über die Paketverwaltung installieren:

```bash
sudo apt install python3-numpy python3-scipy python3-sklearn python3-skimage python3-opencv python3-matplotlib
```

Die Skripte und mehrere Standardpfade setzen den ROS-2-Arbeitsbereich `~/map_ws` voraus. Das Paket wird nach `~/map_ws/src` kopiert, die Skripte in das Wurzelverzeichnis des Arbeitsbereichs. Die folgenden Befehle werden in diesem Ordner ausgeführt:

```bash
mkdir -p ~/map_ws/src
cp -r multi_robot_optimizer ~/map_ws/src/
cp run_batch.py auswertung.py abbildungen_erstellen.py ~/map_ws/
cp -r ergebnisse ~/map_ws/        # optional: Ergebnisse der Arbeit
cd ~/map_ws
colcon build --packages-select multi_robot_optimizer
source install/setup.bash
```

Der Befehl `source install/setup.bash` ist in jedem neuen Terminal nötig. Nach jeder Änderung an Dateien in `src/` muss das Paket neu gebaut werden, da `ros2 run` die installierte Kopie unter `install/` ausführt.


## Packages

### multi_robot_optimizer

Das Paket plant die Formationen der Tracker-Roboter. Es enthält zwei Knoten und das Optimierungsmodul:

| Datei | Inhalt |
|---|---|
| `multi_robot_optimizer/image_to_ros.py` | Kartenknoten: erzeugt die Belegungskarte (aus einem Luftbild oder einem synthetischen Szenario) und die Drohnenposen |
| `multi_robot_optimizer/optimizer_node.py` | Optimierungsknoten: Distanzfeld, äußere Ebene (K-Means, Wahl der Gruppenanzahl), Visualisierung, Protokollierung, Export |
| `multi_robot_optimizer/pso_algorithm.py` | Innere Ebene: Partikelschwarmoptimierung, Messmodell, Straffunktionen; ohne Abhängigkeit von ROS 2 |
| `config/pso_params.yaml` | Parameter des Optimierungsknotens für den Start über die Launch-Datei |
| `launch/optimizer.launch.py` | Startet den Optimierungsknoten mit den Parametern aus `pso_params.yaml` |

Aufruf, Knoten, Topics, Parameter sowie die Ein- und Ausgabeformate beschreibt die [README des Pakets](multi_robot_optimizer/README.md).


## Scripts

### run_batch.py

`run_batch.py` führt ganze Versuchsreihen ohne Eingriff aus. Es startet für jeden Lauf beide Knoten, wartet auf das Ergebnis und fasst am Ende alle Läufe einer Reihe zusammen.

```bash
cd ~/map_ws && source install/setup.bash
python3 run_batch.py test                               # je Szenario ein Lauf
python3 run_batch.py haupt --dry-run                    # nur anzeigen, was laufen würde
nohup python3 run_batch.py haupt typen flotte zeit --workers 3 > lauf.log 2>&1 &
```

Mit `nohup` laufen die Reihen weiter, wenn das Terminal geschlossen wird. Den Fortschritt zeigt `tail -f lauf.log`, abgebrochen wird mit `pkill -INT -f run_batch.py`.

**Eingabe:** Namen einer oder mehrerer Reihen sowie die Optionen `--workers` (Anzahl gleichzeitiger Läufe, Standard 3) und `--dry-run`.

**Vordefinierte Reihen** (in `EXPERIMENTS` im Skript anpassbar):

| Reihe | Zweck | Szenario | Variation | Läufe |
|---|---|---|---|---|
| `test` | Funktionstest | 1 bis 4 | – | 1 |
| `wmove` | Kontrolle von `w_move` | 1 | 0,63 / 1,0 / 1,6 / 2,5 / 4,0 | 30 |
| `wobs` | Kontrolle von `w_obs` | 3 | 0,1 / 1,0 / 10 | 30 |
| `haupt` | Hauptauswertung | 1 bis 4 | – | 30 |
| `flotte` | Roboteranzahl | 1 | 4 / 6 / 8 Roboter | 30 |
| `typen` | Zusammensetzung bei 8 Trackern | 1 | 8A bis 4B | 30 |
| `zeit` | Rechenzeit ohne parallele Last | 1 | 4 / 6 / 8 Roboter | 5 |

Eine Reihe mit eigenen Drohnenposen erhält den Eintrag `drohnen_datei='~/map_ws/drohnenposen.csv'`. Ein auskommentiertes Beispiel steht im Skript.

**Arbeitsweise:**

Die Option `--workers` legt fest, wie viele Läufe gleichzeitig laufen. Jeder Strang erhält eine eigene `ROS_DOMAIN_ID` (ab 40) und eine eigene CSV-Datei. Als Faustregel gilt: Anzahl der Prozessorkerne minus eins. Die Reihe `zeit` läuft immer mit einem Strang, damit parallele Läufe die Rechenzeitmessung nicht verfälschen.

Bereits abgeschlossene Läufe erkennt das Skript an ihrer `Task_ID` und überspringt sie. Nach einem Abbruch setzt ein erneuter Start daher an der Stelle fort. Fehlgeschlagene Läufe wiederholt es einmal. Eine Sperre verhindert, dass zwei Instanzen gleichzeitig laufen.

Werden Code oder Gewichte geändert, muss der Ordner der betroffenen Reihe vorher verschoben oder gelöscht werden. Sonst übernimmt das Skript alte Läufe als erledigt.

**Ausgabe:**

| Pfad | Inhalt |
|---|---|
| `ergebnisse/<reihe>_gesamt.csv` | alle Läufe einer Reihe |
| `ergebnisse/<reihe>/worker_N.csv` | Rohdaten je Strang |
| `ergebnisse/<reihe>/logs/<Task_ID>.log` | Ausgabe beider Knoten je Lauf |
| `ergebnisse/<reihe>/fehler.txt` | fehlgeschlagene Läufe (Timeout, kein CSV-Eintrag) |

Am Ende einer Reihe schreibt das Skript die Gesamtdatei `ergebnisse/<reihe>_gesamt.csv` neu. Die mitgelieferten Ergebnisse der Arbeit sollten daher vor einem eigenen Durchgang gesichert werden.

### auswertung.py

Das Skript fasst eine Gesamtdatei nach Szenario und Flotte zusammen. Es berechnet Erfolgsquote, Mittelwert und Streuung der Gruppenanzahl, die Kostenanteile, die Roboterabstände, die Abstände zu Hindernissen und die Rechenzeit je untersuchter Gruppenanzahl.

```bash
cd ~/map_ws && source install/setup.bash && cd ergebnisse
python3 ~/map_ws/auswertung.py haupt_gesamt.csv
for r in haupt typen flotte zeit; do python3 ~/map_ws/auswertung.py ${r}_gesamt.csv | tee auswertung_${r}.txt; done
```

**Eingabe:** eine Gesamtdatei `<reihe>_gesamt.csv` aus `run_batch.py`.

**Ausgabe:** Kennzahlen als Text im Terminal.

Für die Abstände zu Hindernissen erzeugt das Skript die Karten mit dem Code des installierten Pakets neu, ohne ROS 2 zu starten. Deshalb muss vorher `source install/setup.bash` ausgeführt werden.

### abbildungen_erstellen.py

Das Skript erzeugt die Abbildungen der Arbeit ohne laufendes ROS-System: die Schritte der Kartenerzeugung, die Drohnenposen und die Ergebnisbilder der Hauptauswertung. Die Ergebnisbilder zeigen je Szenario den Lauf mit dem Median der Gesamtkosten.

```bash
cd ~/map_ws
python3 abbildungen_erstellen.py                      # alle Abbildungen
python3 abbildungen_erstellen.py --nur karte posen    # ohne Ergebnisbilder
python3 abbildungen_erstellen.py --csv ~/map_ws/ergebnisse/haupt_gesamt.csv
python3 abbildungen_erstellen.py --format pdf         # Vektorgrafik statt PNG
```

**Eingabe:** die Gesamtdatei der Hauptauswertung (nur für die Ergebnisbilder). Fehlt sie, überspringt das Skript die Ergebnisbilder.

| Option | Standard | Bedeutung |
|---|---|---|
| `--nur` | `karte posen ergebnisse` | Auswahl der Abbildungsgruppen |
| `--ausgabe` | `images` | Zielordner |
| `--format` | `png` | Dateiformat (`png` oder `pdf`) |
| `--szenario-karte` | `4` | Szenario für die Schritte der Kartenerzeugung |
| `--luftbild` | – | eigenes Bild anstelle eines Szenarios für die Kartenerzeugung |
| `--szenario-posen` | `3` | Szenario für die Darstellung der Drohnenposen |
| `--csv` | `~/map_ws/ergebnisse/haupt_gesamt.csv` | Gesamtdatei der Hauptauswertung |
| `--typen`, `--w-move`, `--w-obs` | `AAAABB`, `1.0`, `1.0` | Flotte und Gewichte der Läufe, die das Skript aus der Gesamtdatei auswählt |

**Ausgabe:** Bilddateien im Zielordner.

| Datei | Inhalt |
|---|---|
| `karte_eingang`, `karte_kanten`, `karte_belegung`, `karte_distanzfeld` | Schritte der Kartenerzeugung |
| `drohnenposen` | Drohnenposen um das Messobjekt |
| `ergebnis_s1` bis `ergebnis_s4`, `ergebnis_legende` | Formationen der Hauptauswertung je Szenario und gemeinsame Legende |

Das Skript enthält eine Kopie der Kartenerzeugung und der Posenerzeugung aus `image_to_ros.py`. Änderungen an dieser Datei müssen im Skript nachgezogen werden.


## Reproduzierbarkeit

Der hier abgelegte Codestand entspricht dem Stand, mit dem die Ergebnisse der Arbeit entstanden.

Die Gruppierung ist durch den festen Startwert von K-Means in jedem Lauf gleich. Die Partikelschwarmoptimierung ist stochastisch. Jeder Lauf startet einen neuen Prozess, die Läufe sind daher unabhängige Stichproben. Die Arbeit wertet deshalb 30 Läufe je Konfiguration aus.


## Bekannte Einschränkungen

Das Messmodell bewertet im Wesentlichen die Position eines Referenzpunkts der Drohne. Die Orientierung geht nur über angenommene Winkelunsicherheiten ein. Die Sichtlinienprüfung behandelt Hindernisse als unendlich hoch. Die Bewegungskosten messen den geradlinigen Abstand von der Ausgangsaufstellung und summieren die Wege aller Roboter.

Die weichen Sicherheitsabstände (`d_safe`, `d_min`) werden in verbauten Umgebungen häufig unterschritten. Garantiert sind nur die Kollisionsgrenzen. Die beiden Kollisionsgrenzen gehen von unterschiedlichen Roboterradien aus (0,15 m gegenüber Hindernissen, 0,3 m zwischen Robotern) und sollten für einen realen Einsatz an die tatsächlichen Abmessungen angepasst werden.

Die Erprobung erfolgte kinematisch mit synthetischen Karten. Die Rechenzeit wächst etwa quadratisch mit dem Umfang des Messobjekts.


## Fehlersuche

**Der Optimierungsknoten beendet sich nach einer Minute mit „Keine Daten nach Startfrist“.** Der Kartenknoten wurde zu spät oder gar nicht gestartet. Beide Knoten müssen im selben Netzwerk und mit derselben `ROS_DOMAIN_ID` laufen.

**Eine Änderung im Code wirkt nicht.** Nach Änderungen in `src/` muss neu gebaut werden: `colcon build --packages-select multi_robot_optimizer && source install/setup.bash`. Ob eine bestimmte Änderung installiert ist, zeigt:

```bash
python3 -c "import inspect, multi_robot_optimizer.image_to_ros as m; print('drohnen_quelle' in inspect.getsource(m))"
```

**Ein Parameter wirkt nicht.** Der Name ist vermutlich falsch geschrieben. ROS 2 ignoriert unbekannte Parameter ohne Meldung. Bei Gleitkommaparametern muss der Wert einen Dezimalpunkt enthalten.

**Die Drohnenposen aus der Datei werden nicht übernommen.** Die Ausgabe des Kartenknotens enthält in diesem Fall „Generiere … Dummy-Drohnen“ statt „… Drohnenposen aus … übernommen“. Dann läuft der Knoten im Modus `generiert`. Außerdem prüfen, ob die Datei gespeichert ist (`tail ~/map_ws/drohnenposen.csv`).

**`run_batch.py` meldet „ABBRUCH: Es läuft bereits eine Instanz“.** Eine andere Instanz läuft noch. Anzeigen mit `pgrep -af run_batch.py`, beenden mit `pkill -INT -f run_batch.py`.

**Eine Versuchsreihe rechnet nichts und meldet alle Läufe als vorhanden.** Im Ordner der Reihe liegen noch Ergebnisse eines früheren Durchgangs. Den Ordner verschieben, dann neu starten.

**Die Rechenzeiten unterscheiden sich stark zwischen Reihen.** Parallele Läufe teilen sich die Prozessorkerne. Vergleichbare Rechenzeiten liefert nur die Reihe `zeit`.
