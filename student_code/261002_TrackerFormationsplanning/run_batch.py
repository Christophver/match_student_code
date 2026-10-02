#!/usr/bin/env python3
"""
Versuchsreihen für die Masterarbeit (ROS 2 Jazzy).

Ersetzt run_experiments.py und run_w_move_experiments.py.
- Parallele Läufe: jeder Arbeitsstrang nutzt eine eigene ROS_DOMAIN_ID und eine
  eigene CSV-Datei, damit sich die Topics nicht gegenseitig stören.
- Wiederaufnahme: bereits abgeschlossene Läufe (Task_ID in einer CSV-Datei)
  werden beim erneuten Start übersprungen. Ein Abbruch (Strg+C, Absturz der VM)
  kostet also höchstens die gerade laufenden Läufe.
- Am Ende werden alle CSV-Dateien einer Versuchsreihe zu <reihe>_gesamt.csv
  zusammengeführt.

- Sperre: Es kann immer nur eine Instanz laufen. Eine zweite Instanz bricht
  sofort mit einer Meldung ab (verhindert doppelte Läufe und Überschneidungen
  der ROS_DOMAIN_IDs).

Aufruf (im Terminal, in dem ~/map_ws/install/setup.bash geladen ist):
    python3 run_batch.py test                       # je Szenario ein Lauf (Funktionstest)
    python3 run_batch.py wmove wobs                 # 1. Kontrolle der Gewichte
    python3 run_batch.py haupt typen flotte zeit    # 2. Hauptauswertung, Flotten, Rechenzeit
    python3 run_batch.py haupt --workers 3          # einzelne Reihe
    python3 run_batch.py typen --dry-run            # nur anzeigen, was laufen würde
    python3 run_batch.py tpat tpat_eps tpat_s3      # Voruntersuchung V1 neu (über Nacht)
Reihen mit fest eingestellter Anzahl an Arbeitssträngen (z. B. 'zeit' wegen
der Rechenzeitmessung) ignorieren --workers.
"""
import argparse
import csv
import fcntl
import math
import os
import signal
import subprocess
import sys
import threading
import time
import queue
from pathlib import Path

# ==========================================================================
# KONFIGURATION
# ==========================================================================
OUT_DIR = Path.home() / 'map_ws' / 'ergebnisse'
DOMAIN_BASE = 40          # ROS_DOMAIN_IDs 40, 41, 42, ...
TIMEOUT_S = 3600          # Abbruch eines einzelnen Laufs nach 60 min
TIMER_PERIOD = 0.5        # s je Gruppenanzahl K (beeinflusst das Ergebnis nicht)
STARTUP_WAIT_S = 5.0      # Wartezeit zwischen Optimierungs- und Kartenknoten
D_SAFE = 1.0
D_CRASH = 0.15            # Kollisionsradius gegenüber Hindernissen

def grid_poses(n):
    """Ausgangsaufstellung: zwei Reihen im Abstand von 1 m nahe der Kartenmitte."""
    cols = math.ceil(n / 2)
    poses = []
    for i in range(n):
        poses += [float(i % cols), float(i // cols)]
    return poses


def fleet(types):
    return (grid_poses(len(types)), list(types))


# Flotten: Ausgangsaufstellung und Robotertypen (M = Anzahl der Tracker)
FLEETS = {
    # Variation der Roboteranzahl, immer zwei Roboter vom Typ B
    'N4': fleet('AABB'),        # N = 4, M = 6
    'N6': fleet('AAAABB'),      # N = 6, M = 8  (Standardflotte)
    'N8': fleet('AAAAAABB'),    # N = 8, M = 10
    # Variation der Zusammensetzung bei gleicher Trackeranzahl M = 8
    'A8B0': fleet('AAAAAAAA'),  # N = 8
    'A6B1': fleet('AAAAAAB'),   # N = 7
    'A4B2': fleet('AAAABB'),    # N = 6 (entspricht N6)
    'A2B3': fleet('AABBB'),     # N = 5
    'A0B4': fleet('BBBB'),      # N = 4
}
# Kontrolle: N4, N6 und N8 entsprechen exakt den bisherigen Aufstellungen
assert FLEETS['N6'][0] == [0.0, 0.0, 1.0, 0.0, 2.0, 0.0, 0.0, 1.0, 1.0, 1.0, 2.0, 1.0]

# Versuchsreihen in der Reihenfolge ihrer Priorität
EXPERIMENTS = {
    'test':   dict(scenarios=[1, 2, 3, 4], runs=1,  w_move=[1.6], w_obs=[1.0], fleets=['N6']),
    'haupt':  dict(scenarios=[1, 2, 3, 4], runs=30, w_move=[1.0], w_obs=[1.0], fleets=['N6']),
    # Kontrollreihen für die Gewichte (vor der Hauptauswertung ausführen)
    'wmove':  dict(scenarios=[1], runs=30, w_move=[0.63, 1.0, 1.6, 2.5, 4.0], w_obs=[1.0], fleets=['N6']),
    'wobs':   dict(scenarios=[3], runs=30, w_move=[1.6], w_obs=[0.1, 1.0, 10.0], fleets=['N6']),
    # Roboteranzahl: Schätzgüte, Gruppenanzahl und Kosten (parallel)
    'flotte': dict(scenarios=[1], runs=30, w_move=[1.0], w_obs=[1.0], fleets=['N4', 'N6', 'N8']),
    # Zusammensetzung der Flotte bei gleicher Trackeranzahl (M = 8, parallel)
    'typen':  dict(scenarios=[1], runs=30, w_move=[1.0], w_obs=[1.0],
                   fleets=['A8B0', 'A6B1', 'A4B2', 'A2B3', 'A0B4']),
    # Beispiel für extern berechnete Drohnenposen (CSV: x,y,z[,yaw] je Zeile):
    # 'extern': dict(scenarios=[1], runs=30, w_move=[1.0], w_obs=[1.0], fleets=['N6'],
    #                drohnen_datei='~/map_ws/drohnenposen.csv'),
    # Rechenzeitmessung (Anforderung A7): immer ohne parallele Läufe
    'zeit':   dict(scenarios=[1], runs=5, w_move=[1.0], w_obs=[1.0], fleets=['N4', 'N6', 'N8'],
                   workers=1),
    # Voruntersuchung V1 mit der aktuellen Fassung: Abbruchkriterien der PSO.
    # t_pat = 15 und epsilon = 1e-4 sind die Werte der Hauptauswertung.
    # 'interleave' ordnet die Läufe reihum nach Durchlauf, damit bei einem
    # vorzeitigen Abbruch alle Stufen etwa gleich viele Läufe haben und alle
    # Stufen unter derselben Parallellast rechnen.
    'tpat':     dict(scenarios=[1], runs=30, w_move=[1.0], w_obs=[1.0], fleets=['N6'],
                     t_pat=[5, 10, 15, 20, 30, 40], eps=[1e-4], interleave=True),
    'tpat_eps': dict(scenarios=[1], runs=30, w_move=[1.0], w_obs=[1.0], fleets=['N6'],
                     t_pat=[15], eps=[1e-2, 1e-3, 1e-5], interleave=True),
    'tpat_s3':  dict(scenarios=[3], runs=30, w_move=[1.0], w_obs=[1.0], fleets=['N6'],
                     t_pat=[5, 10, 15, 20, 30, 40], eps=[1e-4], interleave=True),
}
LOCK_FILE = OUT_DIR / '.run_batch.lock'

# ==========================================================================
STOP = threading.Event()
RUNNING = {}              # Arbeitsstrang -> laufende Prozesse
LOCK = threading.Lock()


def fmt_float(v):
    """ROS-2-Parameter müssen als Gleitkommazahl erkennbar sein (1 -> 1.0)."""
    return repr(float(v))


def task_list(name):
    cfg = EXPERIMENTS[name]
    tasks = []
    # t_pat und epsilon nur bei den Reihen zu den Abbruchkriterien; sonst gelten
    # die Standardwerte des Knotens (keine Änderung der bisherigen Task_IDs)
    tps = cfg.get('t_pat', [None])
    epss = cfg.get('eps', [None])
    for sc in cfg['scenarios']:
        for fleet in cfg['fleets']:
            for wm in cfg['w_move']:
                for wo in cfg['w_obs']:
                    for tp in tps:
                        for eps in epss:
                            for r in range(1, cfg['runs'] + 1):
                                tid = f"{name}_S{sc}_{fleet}_wm{wm}_wo{wo}"
                                if tp is not None:
                                    tid += f"_tp{tp}_eps{eps:g}"
                                tid += f"_r{r:02d}"
                                tasks.append(dict(tid=tid, sc=sc, fleet=fleet, wm=wm, wo=wo,
                                                  run=r, tp=tp, eps=eps,
                                                  drohnen_datei=cfg.get('drohnen_datei', '')))
    if cfg.get('interleave'):
        tasks.sort(key=lambda t: t['run'])   # stabil: reihum über alle Stufen
    return tasks


def done_task_ids(exp_dir):
    """Task_IDs aller Läufe, die bereits in einer CSV-Datei stehen."""
    done = set()
    for f in exp_dir.glob('worker_*.csv'):
        with open(f, newline='') as fh:
            for row in csv.DictReader(fh):
                if row.get('Task_ID'):
                    done.add(row['Task_ID'])
    return done


def kill_group(proc, sig=signal.SIGINT, wait=5.0):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, sig)
        proc.wait(timeout=wait)
    except Exception:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except Exception:
            pass


def run_task(task, worker, exp_dir):
    dom = DOMAIN_BASE + worker
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(dom)
    csv_path = exp_dir / f'worker_{worker}.csv'
    log_dir = exp_dir / 'logs'
    log_dir.mkdir(parents=True, exist_ok=True)
    poses, types = FLEETS[task['fleet']]

    opt_cmd = ['ros2', 'run', 'multi_robot_optimizer', 'optimizer_node', '--ros-args',
               '-p', f"w_move:={fmt_float(task['wm'])}",
               '-p', f"w_obs:={fmt_float(task['wo'])}",
               '-p', f"scenario_name:={int(task['sc'])}",
               '-p', f"timer_period:={fmt_float(TIMER_PERIOD)}",
               '-p', f"d_safe:={fmt_float(D_SAFE)}",
               '-p', f"d_crash:={fmt_float(D_CRASH)}",
               '-p', f"start_robot_poses:=[{', '.join(fmt_float(v) for v in poses)}]",
               '-p', f"robot_types:=[{', '.join(repr(t) for t in types)}]",
               '-p', f"csv_path:={csv_path}",
               '-p', f"task_id:={task['tid']}",
               # Unter Volllast startet der Kartenknoten teils erst nach 45 s;
               # die Standardfrist von 60 s reicht dann nicht (Status kein_csv_eintrag)
               '-p', 'startup_timeout:=300.0']
    if task.get('tp') is not None:
        # t_patience ist im Knoten als Ganzzahl deklariert, epsilon als Gleitkommazahl
        opt_cmd += ['-p', f"t_patience:={int(task['tp'])}",
                    '-p', f"epsilon:={fmt_float(task['eps'])}"]
    map_cmd = ['ros2', 'run', 'multi_robot_optimizer', 'image_to_ros', '--ros-args',
               '-p', f"scenario:={int(task['sc'])}"]
    if task.get('drohnen_datei'):
        # Drohnenposen aus Datei statt aus der Kartenerzeugung
        map_cmd += ['-p', 'drohnen_quelle:=datei',
                    '-p', f"drohnen_datei:={os.path.expanduser(task['drohnen_datei'])}"]

    t0 = time.time()
    with open(log_dir / f"{task['tid']}.log", 'w') as log:
        log.write('OPT: ' + ' '.join(opt_cmd) + '\n\n')
        log.flush()
        opt = subprocess.Popen(opt_cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True)
        with LOCK:
            RUNNING[worker] = [opt]
        time.sleep(STARTUP_WAIT_S)
        log.write('\nMAP: ' + ' '.join(map_cmd) + '\n\n')
        log.flush()
        mp = subprocess.Popen(map_cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                              start_new_session=True)
        with LOCK:
            RUNNING[worker] = [opt, mp]
        try:
            opt.wait(timeout=TIMEOUT_S)
            status = 'ok'
        except subprocess.TimeoutExpired:
            status = 'timeout'
            kill_group(opt)
        kill_group(mp)
        with LOCK:
            RUNNING[worker] = []
    time.sleep(1.0)

    # Erfolg prüfen: steht der Lauf in der CSV-Datei?
    if status == 'ok' and task['tid'] not in done_task_ids(exp_dir):
        status = 'kein_csv_eintrag'
    return status, time.time() - t0


def worker_loop(worker, q, exp_dir, stats):
    while not STOP.is_set():
        try:
            task = q.get_nowait()
        except queue.Empty:
            return
        status, dt = run_task(task, worker, exp_dir)
        if status != 'ok' and not STOP.is_set() and task.get('retry', 0) == 0:
            task['retry'] = 1
            q.put(task)                      # einmal wiederholen
        with LOCK:
            stats['n'] += 1
            stats['t'] += dt
            if status != 'ok':
                stats['fail'].append((task['tid'], status))
                with open(exp_dir / 'fehler.txt', 'a') as fh:
                    fh.write(f"{time.strftime('%H:%M:%S')} {task['tid']} {status}\n")
            left = stats['total'] - stats['n']
            eta = stats['t'] / stats['n'] * left / stats['workers'] / 60 if stats['n'] else 0
            print(f"[{time.strftime('%H:%M:%S')}] W{worker} {task['tid']}: {status} "
                  f"({dt:.0f} s) | fertig {stats['n']}/{stats['total']} | Rest ca. {eta:.0f} min",
                  flush=True)


def merge(exp_dir, name):
    rows, header = [], None
    for f in sorted(exp_dir.glob('worker_*.csv')):
        with open(f, newline='') as fh:
            rd = csv.reader(fh)
            h = next(rd, None)
            if h is None:
                continue
            header = header or h
            rows.extend(rd)
    if header:
        out = exp_dir.parent / f'{name}_gesamt.csv'
        with open(out, 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(header)
            w.writerows(rows)
        print(f"Zusammengeführt: {out} ({len(rows)} Läufe)")


def acquire_lock():
    """Stellt sicher, dass nur eine Instanz läuft. Die Sperre gibt das Betriebs-
    system beim Prozessende automatisch frei, auch nach einem Absturz."""
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    fh = open(LOCK_FILE, 'a+')
    try:
        fcntl.flock(fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        fh.seek(0)
        info = fh.read().strip() or 'unbekannt'
        print(f"ABBRUCH: Es läuft bereits eine Instanz von run_batch.py ({info}).\n"
              f"Laufende Instanzen anzeigen: pgrep -af run_batch.py\n"
              f"Beenden:                     pkill -INT -f run_batch.py")
        sys.exit(1)
    fh.seek(0)
    fh.truncate()
    fh.write(f"PID {os.getpid()}, gestartet {time.strftime('%d.%m. %H:%M')}, Reihen: {' '.join(sys.argv[1:])}")
    fh.flush()
    return fh                                # offen halten, solange die Instanz läuft


def run_series(name, workers_arg, dry_run):
    cfg = EXPERIMENTS[name]
    workers = cfg.get('workers', workers_arg)
    exp_dir = OUT_DIR / name
    exp_dir.mkdir(parents=True, exist_ok=True)
    tasks = task_list(name)
    done = done_task_ids(exp_dir)
    todo = [t for t in tasks if t['tid'] not in done]
    print(f"Versuchsreihe '{name}': {len(tasks)} Läufe, davon {len(done & {t['tid'] for t in tasks})} "
          f"bereits vorhanden, {len(todo)} offen. Arbeitsstränge: {workers}", flush=True)
    if dry_run:
        for t in todo[:10]:
            print('  ', t['tid'], '| Typen', ''.join(FLEETS[t['fleet']][1]))
        return
    if not todo:
        merge(exp_dir, name)
        return

    q = queue.Queue()
    for t in todo:
        q.put(t)
    stats = dict(n=0, t=0.0, total=len(todo), workers=workers, fail=[])

    threads = []
    for w in range(workers):
        th = threading.Thread(target=worker_loop, args=(w, q, exp_dir, stats), daemon=True)
        th.start()
        threads.append(th)
        time.sleep(1.0)                      # Starts leicht versetzen
    for th in threads:
        while th.is_alive():
            th.join(timeout=1.0)

    merge(exp_dir, name)
    if stats['fail']:
        print(f"{len(stats['fail'])} Läufe fehlgeschlagen, siehe {exp_dir / 'fehler.txt'}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('reihen', nargs='+', choices=list(EXPERIMENTS))
    ap.add_argument('--workers', type=int, default=3)
    ap.add_argument('--dry-run', action='store_true')
    a = ap.parse_args()

    lock = None if a.dry_run else acquire_lock()

    def on_sigint(sig, frm):
        print("\nAbbruch angefordert – beende laufende Prozesse ...", flush=True)
        STOP.set()
        with LOCK:
            for procs in RUNNING.values():
                for p in procs:
                    kill_group(p, wait=2.0)
    signal.signal(signal.SIGINT, on_sigint)
    signal.signal(signal.SIGTERM, on_sigint)

    for name in a.reihen:
        if STOP.is_set():
            break
        run_series(name, a.workers, a.dry_run)
    print("Fertig.", flush=True)


if __name__ == '__main__':
    main()