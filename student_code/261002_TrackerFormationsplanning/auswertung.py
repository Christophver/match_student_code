#!/usr/bin/env python3
"""
Auswertung der Versuchsreihen für Kapitel 8.

Aufruf (im Ordner ~/map_ws/ergebnisse, nach "source ~/map_ws/install/setup.bash"):
    python3 ~/map_ws/auswertung.py haupt_gesamt.csv
    python3 ~/map_ws/auswertung.py typen_gesamt.csv
    python3 ~/map_ws/auswertung.py flotte_gesamt.csv
    python3 ~/map_ws/auswertung.py zeit_gesamt.csv

Gruppiert nach Szenario und Flotte (Teil der Task_ID) und gibt je Gruppe aus:
Anzahl, Erfolgsquote, K (Mittel, Std., Verteilung), Kostenanteile, C_inter,
kleinster Roboterabstand, Abstände zu Hindernissen und Rechenzeit (gesamt und
je untersuchter Gruppenanzahl).

Die Abstände zu Hindernissen berechnet das Skript mit genau derselben Karte und
demselben Distanzfeld wie der Optimierungsknoten. Dazu führt es die
Kartenerzeugung des installierten Pakets aus, ohne ROS 2 zu starten.
"""
import csv, json, math, sys, types, time, io, contextlib, statistics as st
from collections import Counter, defaultdict

import numpy as np
import scipy.ndimage as ndimage


# --------------------------------------------------------------------------
# Kartenerzeugung ohne ROS: rclpy und Nachrichten durch einfache Platzhalter
# ersetzen und die gesendeten Nachrichten abfangen.
# --------------------------------------------------------------------------
class _Msg:
    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        v = _Msg(); object.__setattr__(self, name, v); return v

_GESENDET = {}
_PARAMS = {}

def _installiere_ros_ersatz():
    class _Param:
        def __init__(self, v): self.value = v
    class _Pub:
        def __init__(self, topic): self.topic = topic
        def publish(self, msg): _GESENDET.setdefault(self.topic, []).append(msg)
    class _Logger:
        def info(self, *a, **k): pass
        warn = warning = error = info
    class _Clock:
        def now(self):
            c = _Msg(); c.to_msg = lambda: _Msg(); return c
    class _Node:
        def __init__(self, name='x'): self._p = {}
        def declare_parameter(self, n, d=None):
            self._p[n] = _PARAMS.get(n, d); return _Param(self._p[n])
        def get_parameter(self, n): return _Param(self._p[n])
        def create_publisher(self, t, topic, q): return _Pub(topic)
        def get_logger(self): return _Logger()
        def get_clock(self): return _Clock()
        def destroy_node(self): pass
    rclpy = types.ModuleType('rclpy')
    rclpy.init = lambda args=None: None
    rclpy.shutdown = rclpy.try_shutdown = lambda: None
    rclpy.ok = lambda: True
    rclpy.create_node = lambda name: _Node(name)
    node_mod = types.ModuleType('rclpy.node'); node_mod.Node = _Node
    rclpy.node = node_mod
    sys.modules['rclpy'] = rclpy; sys.modules['rclpy.node'] = node_mod
    def msgmod(pkg, **cls):
        pk = types.ModuleType(pkg); m = types.ModuleType(pkg + '.msg')
        for k, v in cls.items(): setattr(m, k, v)
        pk.msg = m; sys.modules[pkg] = pk; sys.modules[pkg + '.msg'] = m
    class PoseArray(_Msg):
        def __init__(self): object.__setattr__(self, 'poses', [])
    class Header(_Msg):
        def __init__(self, frame_id='', stamp=None): object.__setattr__(self, 'frame_id', frame_id)
    msgmod('nav_msgs', OccupancyGrid=type('OccupancyGrid', (_Msg,), {}))
    msgmod('geometry_msgs', PoseArray=PoseArray, Pose=type('Pose', (_Msg,), {}),
           Point=type('Point', (_Msg,), {}))
    msgmod('std_msgs', Header=Header, ColorRGBA=type('ColorRGBA', (_Msg,), {}))


_KARTEN = {}

def distanzfeld(szenario):
    """Liefert das Distanzfeld (ESDFMapVectorized) des Szenarios, wie im Knoten."""
    if szenario in _KARTEN:
        return _KARTEN[szenario]
    if not _KARTEN:
        _installiere_ros_ersatz()
    import importlib
    itr = importlib.import_module('multi_robot_optimizer.image_to_ros')
    from multi_robot_optimizer.pso_algorithm import ESDFMapVectorized
    _GESENDET.clear(); _PARAMS['scenario'] = szenario
    schlaf = time.sleep; time.sleep = lambda s: None
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            itr.main()
    finally:
        time.sleep = schlaf
    msg = _GESENDET['/map'][0]
    grid = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    binary = np.where(grid > 50, 1, 0).astype(np.int8)          # wie optimizer_node.py
    esdf = ndimage.distance_transform_edt(1 - binary)
    karte = ESDFMapVectorized(esdf, msg.info.resolution,
                              msg.info.origin.position.x, msg.info.origin.position.y)
    _KARTEN[szenario] = karte
    return karte


# --------------------------------------------------------------------------
def mstd(v):
    if not v:
        return float('nan'), float('nan')
    return st.mean(v), (st.stdev(v) if len(v) > 1 else 0.0)

def min_robot_distance(formations):
    """Kleinster Abstand zweier Roboter innerhalb derselben Formation (m)."""
    dmin = float('inf')
    for f in formations:
        pts = [(f[i], f[i + 1]) for i in range(0, len(f), 2)]
        for a in range(len(pts)):
            for b in range(a + 1, len(pts)):
                dmin = min(dmin, math.dist(pts[a], pts[b]))
    return dmin

def hindernisabstaende(formations, karte):
    """Abstand jedes Roboters jeder Formation zum nächsten Hindernis (m)."""
    xs = np.array([f[i] for f in formations for i in range(0, len(f), 2)])
    ys = np.array([f[i] for f in formations for i in range(1, len(f), 2)])
    d, _ = karte.get_distance_and_angle_batch(xs, ys)
    return np.asarray(d, dtype=float)


def main(path):
    rows = list(csv.DictReader(open(path, newline='')))
    groups = defaultdict(list)
    for r in rows:
        teile = r['Task_ID'].split('_')          # reihe_S1_N6_wm1.0_wo1.0_r01
        groups[(teile[1], teile[2], teile[3], teile[4])].append(r)

    print(f"Datei: {path}  ({len(rows)} Zeilen)\n")
    for key in sorted(groups):
        g = groups[key]
        ok = [r for r in g if r['Status'] == 'abbruchkriterium']
        if not ok:
            print(f"== {' '.join(key)}  | Läufe {len(g)}, kein Lauf mit Abbruchkriterium: "
                  f"{dict(Counter(r['Status'] for r in g))}\n")
            continue
        k = [int(r['Gewaehltes_k']) for r in ok]
        f = lambda s: [float(r[s]) for r in ok]
        crlb, move, obs, inter, ges, zeit = map(f, ['cost_crlb', 'cost_move', 'cost_obs',
                                                    'cost_inter', 'Finale_Kosten_J',
                                                    'Rechenzeit_gesamt_s'])
        dmins = [min_robot_distance(json.loads(r['Formationen'])) for r in ok]
        km, ks = mstd(k)
        print(f"== {' '.join(key)}  | Läufe {len(g)}, davon Abbruchkriterium {len(ok)} "
              f"({100 * len(ok) / len(g):.0f} %), Status: {dict(Counter(r['Status'] for r in g))}")
        print(f"   K*: Mittel {km:.2f}, Std {ks:.2f}, Verteilung {dict(sorted(Counter(k).items()))}")
        for name, v in [('C_ges', ges), ('C_CRLB [µm²]', crlb), ('w_move*C_move', move),
                        ('w_obs*C_obs', obs), ('C_inter', inter), ('Rechenzeit [s]', zeit)]:
            m, s = mstd(v)
            print(f"   {name:<15} Mittel {m:10.2f}  Std {s:9.2f}  Min {min(v):10.2f}  Max {max(v):10.2f}")
        anteil = [c / j * 100 for c, j in zip(crlb, ges) if j]
        print(f"   Anteil CRLB an C_ges: {st.mean(anteil):.1f} %")
        print(f"   C_inter = 0 in {sum(1 for v in inter if v == 0)} von {len(inter)} Läufen; "
              f"kleinster Roboterabstand {min(dmins):.2f} m (Mittel der Minima {st.mean(dmins):.2f} m)")

        # --- NEU: Rechenzeit je untersuchter Gruppenanzahl --------------------
        schritte = [len(json.loads(r['Rechenzeit_Verlauf_s'])) for r in ok]
        je_k = [t / n for t, n in zip(zeit, schritte) if n]
        sm, ss = mstd(schritte)
        jm, js = mstd(je_k)
        print(f"   Untersuchte Gruppenanzahlen je Lauf: Mittel {sm:.2f} (Min {min(schritte)}, Max {max(schritte)})")
        print(f"   Rechenzeit je Gruppenanzahl [s]: Mittel {jm:.1f}  Std {js:.1f}  "
              f"(Variationskoeffizient {100 * js / jm:.0f} %)")

        # --- NEU: Abstände zu Hindernissen ------------------------------------
        szenario = int(key[0].lstrip('S'))
        d_safe = float(ok[0]['d_safe']); d_crash = float(ok[0]['d_crash'])
        try:
            karte = distanzfeld(szenario)
        except Exception as e:
            print(f"   Hindernisabstände: Karte nicht erzeugbar ({e}). "
                  f"Wurde 'source ~/map_ws/install/setup.bash' ausgeführt?\n")
            continue
        alle, minima, ausserhalb = [], [], 0
        for r in ok:
            d = hindernisabstaende(json.loads(r['Formationen']), karte)
            ausserhalb += int(np.sum(d < 0))
            d = d[d >= 0]
            alle.extend(d.tolist())
            if d.size:
                minima.append(float(d.min()))
        alle = np.array(alle)
        print(f"   Hindernisabstand: kleinster {min(minima):.2f} m, Mittel der Minima {st.mean(minima):.2f} m "
              f"(Roboterstandorte gesamt: {alle.size})")
        print(f"   Standorte mit d < d_safe ({d_safe} m): {100 * np.mean(alle < d_safe):.1f} %; "
              f"mit d <= d_crash ({d_crash} m): {int(np.sum(alle <= d_crash))}; "
              f"außerhalb der Karte: {ausserhalb}")
        print()


if __name__ == '__main__':
    main(sys.argv[1])