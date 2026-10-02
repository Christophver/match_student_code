import rclpy
from rclpy.node import Node
import numpy as np
import math
import time
import json
import matplotlib
matplotlib.use('Agg')  # KORREKTUR: kein Grafikfenster im Batch-Betrieb nötig
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
import scipy.ndimage as ndimage
import csv
import os
import sys


# ROS 2 Messages
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseArray

# Eigene Imports
from multi_robot_optimizer.pso_algorithm import SwarmOptimizer, ESDFMapVectorized, C_BASE, P_CRASH

class OptimizerNode(Node):
    def __init__(self):
        super().__init__('formation_optimizer_node')

        # ==========================================
        # Schritt 1: Dateneingabe & Parameter
        # ==========================================
        # KORREKTUR: w_crlb und max_drone_dist entfernt; beide hatten keine Wirkung
        # (Schätzgüte geht ungewichtet ein, d_max ist in pso_algorithm.py festgelegt).
        self.declare_parameter('w_move', 1.0)   # festgelegt über Kontrollreihe K1
        self.declare_parameter('w_obs', 1.00)
        # KORREKTUR: sechs Einträge passend zur Ausgangsaufstellung (vorher wurden
        # zwei Roboter stillschweigend mit Typ A ergänzt)
        self.declare_parameter('robot_types', ['A', 'A', 'A', 'A', 'B', 'B'])
        self.declare_parameter('max_iterations', 50)
        self.declare_parameter('c1', 1.5)
        self.declare_parameter('c2', 1.5)
        self.declare_parameter('inertia_weight', 0.5)
        self.declare_parameter('d_safe', 1.0)
        self.declare_parameter('d_crash', 0.15)
        self.declare_parameter('t_patience', 15)
        self.declare_parameter('epsilon', 0.0001)

        # NEU: Parameter für die automatisierten Versuchsreihen
        # Periode des Zeitgebers (je Aufruf eine Gruppenanzahl K). Beeinflusst das
        # Ergebnis nicht, nur die Wartezeit zwischen zwei Schritten.
        self.declare_parameter('timer_period', 4.0)
        # Ausgabedatei und Kennung des Laufs (für parallele Läufe und Wiederaufnahme)
        self.declare_parameter('csv_path', os.path.expanduser('~/map_ws/pso_evaluation_results.csv'))
        self.declare_parameter('task_id', '')
        # NEU: Export der finalen Lösung. Ist ein Pfad gesetzt, schreibt der Knoten
        # am Ende die Roboterpositionen je Formation in diese CSV-Datei und die
        # Zuordnung der Drohnenposen zu den Formationen in <Name>_drohnen.csv.
        self.declare_parameter('formation_export', '')
        # Ausgangsaufstellung der Roboter [x1, y1, x2, y2, ...] in m
        self.declare_parameter('start_robot_poses', [0.0, 0.0, 1.0, 0.0, 2.0, 0.0,
                                                     0.0, 1.0, 1.0, 1.0, 2.0, 1.0])

        # ==========================================
        # STATISTIK-MODUS SCHALTER (An/Aus)
        # ==========================================
        self.declare_parameter('batch_mode', True)
        self.enable_batch_evaluation = bool(self.get_parameter('batch_mode').value)
        self.target_name = "Objekt_1"
        self.current_run = 1
        self.total_runs = 1            # Anzahl der Durchläufe im Batch-Modus
        self.experiment_results = []  
        self.declare_parameter('scenario_name', 1)       # Speicher für die CSV-Daten
        # ==========================================
        
        params = {
            'w_move': self.get_parameter('w_move').value,
            'w_obs': self.get_parameter('w_obs').value,
            'robot_types': self.get_parameter('robot_types').value,
            'max_iterations': self.get_parameter('max_iterations').value,
            'c1': self.get_parameter('c1').value,
            'c2': self.get_parameter('c2').value,
            'inertia_weight': self.get_parameter('inertia_weight').value,
            'd_safe': self.get_parameter('d_safe').value,
            'd_crash': self.get_parameter('d_crash').value,
            't_patience' : self.get_parameter('t_patience').value,
            'epsilon' : self.get_parameter('epsilon').value,
        }

        self.optimizer = SwarmOptimizer(params)
        
        # ROS Publisher für die RViz-Visualisierung
        self.marker_pub = self.create_publisher(MarkerArray, 'formation_markers', 10)
        
        # ==========================================
        # Schritt 2: Subscriber & Initialisierung
        # ==========================================
        self.all_drones = np.array([])  # Wird über das Topic gefüllt
        
        self.map_received = False
        self.drones_received = False
        
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.drone_sub = self.create_subscription(PoseArray, '/drone_targets', self.drone_callback, 10)

        # Farben für die Cluster-Visualisierung
        self.cluster_colors = [
            (0.122, 0.467, 0.706), (0.682, 0.780, 0.910), (1.000, 0.498, 0.055),
            (1.000, 0.733, 0.471), (0.173, 0.627, 0.173), (0.596, 0.875, 0.541),
            (0.839, 0.153, 0.157), (1.000, 0.596, 0.588), (0.580, 0.404, 0.741),
            (0.773, 0.690, 0.835), (0.549, 0.337, 0.294), (0.769, 0.612, 0.580),
            (0.890, 0.467, 0.761), (0.969, 0.714, 0.824), (0.498, 0.498, 0.498),
            (0.780, 0.780, 0.780), (0.737, 0.741, 0.133), (0.859, 0.859, 0.553),
            (0.090, 0.745, 0.812), (0.620, 0.855, 0.898),
        ]

        # Initiale Posen der Bodenroboter (jetzt als ROS-2-Parameter)
        self.start_robot_poses = [float(v) for v in self.get_parameter('start_robot_poses').value]
        self.timer_period = float(self.get_parameter('timer_period').value)
        self.csv_path = os.path.expanduser(self.get_parameter('csv_path').value)
        self.task_id = str(self.get_parameter('task_id').value)
        self.formation_export = os.path.expanduser(str(self.get_parameter('formation_export').value))

        # NEU: Flottengröße einmalig prüfen (vorher im SwarmOptimizer mit Absturz)
        n_robots = len(self.start_robot_poses) // 2
        types = list(params['robot_types']) + ['A'] * max(0, n_robots - len(params['robot_types']))
        self.num_trackers = sum(2 if t == 'B' else 1 for t in types[:n_robots])
        if self.num_trackers < 6:
            self.get_logger().error(
                f"Flotte mit {self.num_trackers} Trackern ist zu klein (mindestens 6). "
                "Jede Gruppe würde als nicht erfassbar gelten.")
        
        # Initialisierung der PSO-Zustandsvariablen
        self.current_k = 1
        self.optimization_done = False 
        self.k_history = []
        self.total_cost_history = []
        self.cluster_costs_history = {}
        self.previous_total_cost = float('inf')
        self.best_marker_array = None 

        # NEU: Protokollgrößen für die CSV-Datei
        self.runtime_history = []       # Rechenzeit je untersuchter Gruppenanzahl (s)
        self.best_run_inter = 0.0
        self.best_run_formations = []
        self.best_run_group_sizes = []
        self.best_run_clusters = []         # NEU: Drohnenposen je Gruppe der besten Lösung
        
        self.timer = None

        # NEU: Wachhund für den Start. Verpasst der Knoten Karte oder Drohnenposen
        # (z. B. weil der Start unter Last länger dauert), beendet er sich nach
        # startup_timeout Sekunden selbst. Das Versuchsskript wiederholt den Lauf
        # dann sofort, statt bis zu seinem Zeitlimit zu warten.
        self.declare_parameter('startup_timeout', 60.0)
        self.startup_deadline = time.time() + float(self.get_parameter('startup_timeout').value)
        self.watchdog_timer = self.create_timer(5.0, self.startup_watchdog)
        self.get_logger().info("Warte auf Karte (/map) und Drohnen (/drone_targets)...")
        
        # Prüft sofort, ob evtl. schon Daten anliegen (für schnelle Neustarts)
        self.check_start_condition()

    def map_callback(self, msg):
        """Wird aufgerufen, sobald image_to_ros.py die Karte publiziert."""
        if self.map_received:
            return  # Nur beim ersten Mal ausführen

        self.get_logger().info("Karte empfangen! Verarbeite Grid und berechne ESDF...")

        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        width = msg.info.width
        height = msg.info.height

        # 1. ROS 1D-Array zurück in ein 2D NumPy Array wandeln
        grid_1d = np.array(msg.data, dtype=np.int8)
        grid_2d = grid_1d.reshape((height, width))

        # 2. Binäres Grid für den Bresenham LoS-Check erstellen (0 = Frei, 1 = Wand)
        raw_binary = np.where(grid_2d > 50, 1, 0)
        # NEU: Zwingt das Array in einen sauberen C-Speicherblock für Numba!
        self.binary_grid = np.ascontiguousarray(raw_binary, dtype=np.int8)

        # 3. ESDF berechnen (Distanz zu Wänden)
        free_space = 1 - self.binary_grid
        self.esdf_matrix = ndimage.distance_transform_edt(free_space)

        # 4. Deine neue vektorisierte Klasse initialisieren!
        self.esdf_map = ESDFMapVectorized(self.esdf_matrix, self.map_resolution, self.map_origin_x, self.map_origin_y)

        # 5. Dem Optimizer die Map-Objekte injizieren
        self.optimizer.esdf_map = self.esdf_map
        self.optimizer.binary_grid = self.binary_grid
        self.optimizer.map_resolution = self.map_resolution
        self.optimizer.map_origin_x = self.map_origin_x
        self.optimizer.map_origin_y = self.map_origin_y

        self.map_received = True
        self.get_logger().info("Umgebung erfolgreich initialisiert. Erzeuge RViz Debug-Ansicht...")

        # 1. Debug Map dauerhaft an RViz senden (alle 2 Sekunden)
        # So kann RViz die Nachricht nicht mehr verpassen!
        self.debug_timer = self.create_timer(2.0, self.publish_rviz_debug_map)

        # KORREKTUR: Der Zeitgeber wurde hier zusätzlich unbedingt erzeugt. Kamen die
        # Drohnenposen vor der Karte an, liefen zwei Zeitgeber parallel. Kam die
        # Karte zuerst und feuerte der Zeitgeber vor dem Empfang der Posen, beendete
        # sich der Knoten (K = 1 > N_d = 0). Jetzt startet nur check_start_condition
        # den Zeitgeber, und zwar erst, wenn beide Daten vorliegen.
        self.check_start_condition()
        
    def check_start_condition(self):
        """Startet die PSO-Schleife erst, wenn Karte UND Drohnen-Posen existieren."""
        if self.map_received and self.drones_received and self.timer is None:
            self.get_logger().info("🚀 Beide Datenströme bereit. Starte Optimierungsschleife...")
            # Optional: Hier wieder dein rviz debug grid aktivieren falls gewünscht
            self.timer = self.create_timer(self.timer_period, self.evaluate_next_k)

    def startup_watchdog(self):
        """Beendet den Knoten, wenn Karte oder Drohnenposen nicht rechtzeitig eintreffen."""
        if self.map_received and self.drones_received:
            self.watchdog_timer.cancel()
            return
        if time.time() > self.startup_deadline:
            self.get_logger().error(
                f"Keine Daten nach Startfrist (Karte: {self.map_received}, "
                f"Drohnenposen: {self.drones_received}). Knoten beendet sich.")
            import sys
            sys.exit(3)

    def drone_callback(self, msg):
        """Empfängt die maßgeschneiderten Drohnen-Positionen direkt von OpenCV."""
        if self.drones_received:
            return

        drones_6d = []
        for pose in msg.poses:
            dx = pose.position.x
            dy = pose.position.y
            dz = pose.position.z
            
            # Rekonstruktion des Yaw aus dem Quaternion
            yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
            drones_6d.append([dx, dy, dz, 0.0, 0.0, yaw])

        self.all_drones = np.array(drones_6d)

        # NEU: Leere Nachricht ignorieren (sonst gälte sofort K = 1 > N_d = 0)
        if len(self.all_drones) == 0:
            self.get_logger().warn("Leere Drohnenliste empfangen, warte auf nächste Nachricht.")
            return

        self.drones_received = True
        self.get_logger().info(f"✅ {len(self.all_drones)} Ziel-Drohnen erfolgreich registriert!")
        
        # Prüfen, ob wir loslegen können
        self.check_start_condition()

    


    def evaluate_next_k(self):
        
        self.publish_drone_targets()

        start_time = time.perf_counter()

        


        if self.optimization_done:
            if self.best_marker_array is not None:
                self.marker_pub.publish(self.best_marker_array)
            return
        
        if self.current_k > len(self.all_drones):
            # KORREKTUR: Vorher beendete sich der Knoten hier ohne finish_run(), und
            # der Lauf fehlte in der CSV-Datei. Jetzt wird die letzte zulässige
            # Gruppenanzahl als Ergebnis gespeichert (Status 'max_k'), oder der Lauf
            # wird als 'keine_loesung' protokolliert.
            self.get_logger().info("Maximale Cluster-Anzahl erreicht. Abbruch.")
            if math.isfinite(self.previous_total_cost):
                self.finish_run(final_k=self.current_k - 1,
                                final_cost=self.previous_total_cost, status='max_k')
            else:
                self.finish_run(final_k=0, final_cost=float('nan'), status='keine_loesung')
            return

        self.get_logger().info(f"\n------------------------------------------------")
        self.get_logger().info(f"Starte Evaluierung für k = {self.current_k}")
        
        clusters = []
        labels = []  # NEU: Hier speichern wir die Zuweisungen für RViz
            

        if self.current_k == 1:
            clusters.append(self.all_drones.tolist())
            # Bei k=1 sind alle Drohnen im selben Cluster (Index 0)
            labels = [0] * len(self.all_drones) 
        else:
            # PRO-TIPP für deine Arbeit: Nur über X, Y, Z clustern! ([:, :3])
            # Winkel (Yaw) haben eine andere Skalierung als Meter und würden das Clustering verfälschen.
            kmeans = KMeans(n_clusters=self.current_k, random_state=42, n_init=10).fit(self.all_drones[:, :3])
            
            labels = kmeans.labels_  # Das ist unser Array für RViz!
            
            for i in range(self.current_k):
                clusters.append(self.all_drones[kmeans.labels_ == i].tolist())

        self.publish_drone_targets(labels=labels)
                
        current_total_cost = 0.0
        formations = []
        current_cluster_costs = [] 
        valid_solution = True

        # NEU: Sammler für die Einzelkosten dieses Durchlaufs
        total_crlb_cost = 0.0
        total_obs_cost = 0.0
        total_move_cost = 0.0
        total_inter_cost = 0.0
        
        
        # Dynamisch w_move auslesen, sonst 0.1
        w_move = self.get_parameter('w_move').value if self.has_parameter('w_move') else 0.1

        # w_obs dynamisch auslesen und an PSO senden
        w_obs = self.get_parameter('w_obs').value if self.has_parameter('w_obs') else 1.0
        self.optimizer.params['w_obs'] = w_obs

        # Finaler Name für die Auswertung

        scenario_idx = self.get_parameter('scenario_name').value
        self.target_name = f"Szenario_{scenario_idx}_Final"
        # ==========================================
        
        for idx, cluster_drones in enumerate(clusters):
            
            # ACHTUNG: Aufruf geändert! obstacle_coords wird nicht mehr an run_pso übergeben!
            best_form, pso_cost = self.optimizer.run_pso(cluster_drones, self.start_robot_poses)
            
            if best_form is None or pso_cost >= C_BASE:
                self.get_logger().warn(f"PSO für Cluster {idx+1} gescheitert (Sichtlinie blockiert oder Singularität).")
                valid_solution = False
                break 

            # KORREKTUR: Kollisionen (1e6) und Positionen außerhalb der Karte (1e7)
            # lagen unter C_base und wurden bisher als zulässig akzeptiert. Die
            # Prüfung nutzt die ungewichteten Strafen, damit sie auch bei w_obs = 0 greift.
            if self.optimizer.best_obs_raw >= P_CRASH or self.optimizer.best_inter >= P_CRASH:
                self.get_logger().warn(f"PSO für Cluster {idx+1} gescheitert (Kollision oder außerhalb der Karte).")
                valid_solution = False
                break
                
            c_move_raw = 0.0
            num_robots_in_form = len(best_form) // 2
            
            for i in range(num_robots_in_form):
                sx = self.start_robot_poses[i*2]
                sy = self.start_robot_poses[i*2+1]
                tx = best_form[i*2]
                ty = best_form[i*2+1]
                c_move_raw += math.hypot(tx - sx, ty - sy)
                
            
            
            cluster_total_cost = pso_cost + (w_move * c_move_raw)
            current_total_cost += cluster_total_cost
            formations.append(best_form)
            current_cluster_costs.append(cluster_total_cost)

            # NEU: Werte für die CSV aufaddieren (über alle Cluster hinweg)
            move_cost = w_move * c_move_raw
            total_crlb_cost += self.optimizer.best_crlb
            total_obs_cost += self.optimizer.best_obs
            total_inter_cost += self.optimizer.best_inter
            total_move_cost += move_cost
            

        end_time = time.perf_counter()
        elapsed_time = end_time - start_time
        self.runtime_history.append(round(elapsed_time, 4))  # NEU: für die CSV-Datei
        self.get_logger().info(f"⏱️ Evaluierung für k={self.current_k} abgeschlossen in {elapsed_time:.3f} Sekunden.")

        if not valid_solution:
            self.get_logger().warn(f"--> k={self.current_k} ist physikalisch ungültig. Erhöhe k erzwungenermaßen.")
            self.previous_total_cost = float('inf') 
            self.current_k += 1
            return 
        
        self.get_logger().info(f"Schritt 5: Gesamtkosten J_total = {current_total_cost:.4f}")

        self.k_history.append(self.current_k)
        self.total_cost_history.append(current_total_cost)
        self.cluster_costs_history[self.current_k] = current_cluster_costs
        
        if self.current_k > 1 and current_total_cost > self.previous_total_cost:
            self.get_logger().info("--> JA (Stopp)")
            self.get_logger().info(f"    (Aktuell: {current_total_cost:.4f} > Vorher: {self.previous_total_cost:.4f})")
            
            self.get_logger().info(f"\n+++ ERGEBNIS +++")
            self.get_logger().info(f"Beste Formation ermittelt für k = {self.current_k - 1} Cluster.")
            
            # ===== NEUER BATCH-LOGIK BLOCK =====
            if self.enable_batch_evaluation:
                # Schalter ist AN: Daten des besten k speichern und Loop neu starten.
                # WICHTIG: plot_results() wird hier übersprungen, damit der Code nicht pausiert!
                self.finish_run(final_k=self.current_k - 1, final_cost=self.previous_total_cost, status='abbruchkriterium')
                return 
            else:
                # Schalter ist AUS: Normales Verhalten für Vorführungen
                self.optimization_done = True 
                self.export_final_solution(self.current_k - 1, self.previous_total_cost)
                self.plot_results() 
                return
            # ===================================

        self.get_logger().info("--> NEIN, Erhöhe k = k + 1")
        
        marker_array = MarkerArray()

        if self.best_marker_array is not None:
            self.marker_pub.publish(self.best_marker_array)
                
        
        
        sorted_clusters = []
        for c_drones, form in zip(clusters, formations):
            center = np.mean(c_drones, axis=0)
            angle = np.arctan2(center[1] - 5.0, center[0] - 5.0)
            sorted_clusters.append((angle, c_drones, form))
            
        sorted_clusters.sort(key=lambda item: item[0])

        for idx, (_, cluster_drones, form) in enumerate(sorted_clusters):
            c_color = self.cluster_colors[idx % len(self.cluster_colors)]
            self.add_cluster_markers(marker_array, cluster_drones, form, 
                                     drone_color=c_color, robot_color=c_color, base_id=(idx+1)*100)
        
        self.best_marker_array = marker_array 
        self.marker_pub.publish(marker_array)

        self.previous_total_cost = current_total_cost

        # === HIER IST DER FIX: Die Werte für das aktuell beste k "einfrieren" ===
        self.best_run_crlb = total_crlb_cost
        self.best_run_move = total_move_cost
        self.best_run_obs = total_obs_cost
        self.best_run_inter = total_inter_cost
        self.best_run_formations = [[round(float(v), 4) for v in f] for f in formations]
        self.best_run_group_sizes = [len(c) for c in clusters]
        self.best_run_clusters = [[[float(v) for v in d] for d in c] for c in clusters]

        self.current_k += 1

    


    def publish_drone_targets(self, labels=None):
        """Sendet die Drohnen an RViz und färbt sie nach Cluster-Zugehörigkeit (k)."""
        marker_array = MarkerArray()
        
        for idx, d in enumerate(self.all_drones):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "drone_targets"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(d[0])
            marker.pose.position.y = float(d[1])
            marker.pose.position.z = float(d[2])
            marker.pose.orientation.w = 1.0
            
            # Form und Größe
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            
            # --- FARB-LOGIK ---
            if labels is not None and len(labels) == len(self.all_drones):
                # Wir haben eine Cluster-Zuweisung für diese Drohne!
                cluster_id = int(labels[idx])
                c_color = self.cluster_colors[cluster_id % len(self.cluster_colors)]
                
                marker.color.r = float(c_color[0])
                marker.color.g = float(c_color[1])
                marker.color.b = float(c_color[2])
                marker.color.a = 1.0 
            else:
                # Fallback (z.B. am Anfang, wenn noch nicht geclustert wurde)
                marker.color.r = 0.5
                marker.color.g = 0.5
                marker.color.b = 0.5
                marker.color.a = 0.8
            
            marker_array.markers.append(marker)

        if not hasattr(self, 'drone_target_pub'):
            self.drone_target_pub = self.create_publisher(MarkerArray, 'drone_targets_rviz', 10)
            
        self.drone_target_pub.publish(marker_array)

    def add_cluster_markers(self, marker_array, drones, formation, drone_color, robot_color, base_id):
        for idx, d in enumerate(drones):
            drone_marker = self.create_base_marker(base_id + idx, Marker.SPHERE, d[0], d[1], d[2], *drone_color)
            drone_marker.scale.x, drone_marker.scale.y, drone_marker.scale.z = 0.5, 0.5, 0.5
            marker_array.markers.append(drone_marker)
            
        robot_types = self.get_parameter('robot_types').value if self.has_parameter('robot_types') else ['A', 'A', 'A', 'A', 'B', 'B']

        num_robots = len(formation) // 2
        for i in range(num_robots):
            rx = float(formation[i*2])
            ry = float(formation[i*2+1])
            r_type = robot_types[i] if i < len(robot_types) else 'A'

            if r_type == 'A':
                scale_z = 0.2
                robot_marker = self.create_base_marker(base_id + 50 + i, Marker.CYLINDER, rx, ry, scale_z / 2.0, *robot_color)
                robot_marker.scale.x, robot_marker.scale.y, robot_marker.scale.z = 0.5, 0.5, scale_z
            else:
                scale_z = 1.2
                robot_marker = self.create_base_marker(base_id + 50 + i, Marker.CYLINDER, rx, ry, scale_z / 2.0, *robot_color)
                robot_marker.scale.x, robot_marker.scale.y, robot_marker.scale.z = 0.6, 0.6, scale_z

            marker_array.markers.append(robot_marker)

    def create_base_marker(self, m_id, m_type, x, y, z, r, g, b):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pso_formation"
        marker.id = m_id
        marker.type = m_type
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = 1.0
        return marker
    
    def plot_results(self):
        plt.figure(figsize=(10, 6))
        plt.plot(self.k_history, self.total_cost_history, 'k-o', linewidth=2, label=r'Gesamtkosten ($J_{total}$)')
        
        for k, costs in self.cluster_costs_history.items():
            for idx, c in enumerate(costs):
                c_color = self.cluster_colors[idx % len(self.cluster_colors)]
                offset = (idx - len(costs)/2) * 0.05 
                plt.scatter(k + offset, c, s=100, zorder=5, color=c_color)
                plt.text(k + offset + 0.05, c, f'C{idx+1}', fontsize=9, verticalalignment='center')

        best_k = self.current_k - 1
        plt.axvline(x=best_k, color='green', linestyle='--', alpha=0.5, label=f'Optimales k = {best_k}')

        plt.title(r'Entwicklung der Gesamtkosten ($J_{total}$) über die Iterationen')
        plt.xlabel('Anzahl der Cluster (k)')
        plt.ylabel(r'Kosten ($J_{total}$)')
        plt.xticks(self.k_history) 
        plt.grid(True, linestyle=':', alpha=0.7)
        plt.legend()
        plt.tight_layout()
        
        save_path = '/home/vboxuser/map_ws/j_total_plot.png'
        plt.savefig(save_path, dpi=300)
        self.get_logger().info(f"Plot wurde erfolgreich gespeichert unter: {save_path}")
        plt.show()



    def publish_rviz_debug_map(self):
        """Erzeugt eine farbige Kachel-Karte für RViz zur Überprüfung des ESDFs."""
        # Wir fassen 5x5 Pixel zusammen (Downsampling), damit RViz nicht abstürzt
        step = 5 
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "esdf_gradient"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD

        # Größe einer Kachel im Raum
        marker.scale.x = float(self.map_resolution * step)
        marker.scale.y = float(self.map_resolution * step)
        marker.scale.z = 0.05

        points = []
        colors = []

        # Maximalen Distanzwert für die Skalierung des Grüns finden
        max_dist = float(np.max(self.esdf_matrix))
        if max_dist == 0: max_dist = 1.0

        for y in range(0, self.binary_grid.shape[0], step):
            for x in range(0, self.binary_grid.shape[1], step):
                dist = self.esdf_matrix[y, x]
                is_obstacle = self.binary_grid[y, x] > 0

                p = Point()
                # Weltkoordinate für die Kachel berechnen
                p.x = float(self.map_origin_x + (x + step/2.0) * self.map_resolution)
                p.y = float(self.map_origin_y + (y + step/2.0) * self.map_resolution)
                p.z = 0.0
                
                c = ColorRGBA()
                c.a = 0.9 # volle deckkraft
                
                if is_obstacle:
                    # Hindernis = Rot
                    c.r, c.g, c.b = 1.0, 0.0, 0.0
                else:
                    # ESDF = Grüner Gradient! (Je höher die Distanz, desto grüner)
                    intensity = dist / max_dist
                    c.r = 0.0
                    c.g = float(intensity) 
                    c.b = 0.0

                points.append(p)
                colors.append(c)

        marker.points = points
        marker.colors = colors

        if not hasattr(self, 'debug_pub'):
            self.debug_pub = self.create_publisher(Marker, 'debug_grid', 1)
            
        self.debug_pub.publish(marker)
        self.get_logger().info("✅ Debug-Grid (Grüner Gradient) an RViz gesendet!")

    def calculate_drone_pose_facing_wall(self, drone_pos, obs):
        if isinstance(obs, dict):
            cx, cy = obs.get('x', 0.0), obs.get('y', 0.0)
            sx, sy = obs.get('sx', obs.get('s', 2.0)), obs.get('sy', obs.get('s', 2.0))
            sz = obs.get('h', 5.0)
            cz = obs.get('z', sz / 2.0)
        else:
            if len(obs) == 6:
                cx, cy, cz, sx, sy, sz = obs
            else:
                cx, cy = obs[0], obs[1]
                sx, sy, sz = 2.0, 2.0, 5.0
                cz = sz / 2.0

        dx, dy, dz = drone_pos[0], drone_pos[1], drone_pos[2]

        x_surf = max(cx - sx/2.0, min(dx, cx + sx/2.0))
        y_surf = max(cy - sy/2.0, min(dy, cy + sy/2.0))
        z_surf = max(cz - sz/2.0, min(dz, cz + sz/2.0))

        vx = x_surf - dx
        vy = y_surf - dy
        vz = z_surf - dz

        if vx == 0 and vy == 0 and vz == 0:
            vx, vy, vz = cx - dx, cy - dy, cz - dz

        yaw = math.atan2(vy, vx)
        pitch = math.atan2(vz, math.hypot(vx, vy))
        roll = 0.0 

        return [dx, dy, dz, roll, pitch, yaw]
    



    def export_final_solution(self, final_k, final_cost):
        """Schreibt die Roboterpositionen der finalen Lösung und die Zuordnung der
        Drohnenposen zu den Formationen als CSV-Dateien."""
        if not self.formation_export or not self.best_run_formations:
            return
        pfad = self.formation_export
        os.makedirs(os.path.dirname(pfad) or '.', exist_ok=True)
        stamm, endung = os.path.splitext(pfad)
        pfad_drohnen = f"{stamm}_drohnen{endung or '.csv'}"

        n_robots = len(self.start_robot_poses) // 2
        types = list(self.optimizer.params.get('robot_types', []))
        types = (types + ['A'] * n_robots)[:n_robots]
        kopf = (f"# Szenario {self.target_name}, Task {self.task_id or '-'}, "
                f"K = {final_k}, C_ges = {final_cost:.2f}, "
                f"w_move = {self.get_parameter('w_move').value}, "
                f"w_obs = {self.optimizer.params.get('w_obs')}\n")
        try:
            with open(pfad, 'w', encoding='utf-8') as fh:
                fh.write(kopf)
                fh.write("gruppe,roboter,typ,x,y,start_x,start_y\n")
                for g, form in enumerate(self.best_run_formations, start=1):
                    for i in range(n_robots):
                        fh.write(f"{g},{i + 1},{types[i]},{form[2 * i]:.4f},{form[2 * i + 1]:.4f},"
                                 f"{self.start_robot_poses[2 * i]:.4f},{self.start_robot_poses[2 * i + 1]:.4f}\n")
            with open(pfad_drohnen, 'w', encoding='utf-8') as fh:
                fh.write(kopf)
                fh.write("gruppe,x,y,z,yaw\n")
                for g, cluster in enumerate(self.best_run_clusters, start=1):
                    for d in cluster:
                        fh.write(f"{g},{d[0]:.4f},{d[1]:.4f},{d[2]:.4f},{d[5]:.4f}\n")
            self.get_logger().info(f"Finale Lösung gespeichert: {pfad} und {pfad_drohnen}")
        except OSError as e:
            self.get_logger().error(f"Export der finalen Lösung fehlgeschlagen: {e}")

    def finish_run(self, final_k, final_cost, status='abbruchkriterium'):
        """Speichert die Daten des aktuellen Laufs inklusive k-Historie und triggert den nächsten."""
        
        val_crlb = getattr(self, 'best_run_crlb', 0.0)
        val_move = getattr(self, 'best_run_move', 0.0)
        val_obs = getattr(self, 'best_run_obs', 0.0)
        
        # NEU: Zeitgeber anhalten, damit kein weiterer Schritt mehr startet
        if self.timer is not None:
            self.timer.cancel()

        # NEU: finale Lösung als CSV ausgeben (falls formation_export gesetzt)
        if status != 'keine_loesung':
            self.export_final_solution(final_k, final_cost)
        
        self.experiment_results.append({
            'task_id': self.task_id,
            'target': self.target_name,
            'run': self.current_run,
            'status': status,
            'k': final_k,
            'cost': final_cost,
            'cost_crlb': val_crlb,
            'cost_move': val_move,
            'cost_obs': val_obs,
            'cost_inter': self.best_run_inter,
            'k_history': list(self.k_history),
            'cost_history': list(self.total_cost_history),
            'runtime_history': list(self.runtime_history),
            'runtime_total': round(float(sum(self.runtime_history)), 4),
            'n_drones': int(len(self.all_drones)),
            'group_sizes': list(self.best_run_group_sizes),
            'formations': list(self.best_run_formations),
        })
        
        self.get_logger().info(f"🏁 --- Durchlauf {self.current_run}/{self.total_runs} abgeschlossen! (k={final_k}, J={final_cost:.2f}) ---")
        
        if self.current_run < self.total_runs:
            self.current_run += 1
            self.get_logger().info(f"\n================================================")
            self.get_logger().info(f"🚀 STARTE DURCHLAUF {self.current_run} VON {self.total_runs}")
            self.get_logger().info(f"================================================\n")
            
            self.current_k = 1
            self.previous_total_cost = float('inf')
            self.best_marker_array = None 
            self.optimization_done = False
            self.k_history = []
            self.total_cost_history = []
            self.cluster_costs_history = {}
            self.runtime_history = []
            
            self.timer.reset()
        else:
            self.get_logger().info(f"🎉 Alle {self.total_runs} Durchläufe beendet! Werte Statistik aus...")
            self.evaluate_statistics_and_shutdown()
    # reset_for_next_run bleibt exakt so wie es ist! (Dort setzt du die Listen ja schon auf [] zurück)

    def reset_for_next_run(self):
        """Setzt die Variablen zurück und startet die Schleife von vorn."""
        self.current_k = 1  
        self.previous_total_cost = float('inf')
        self.k_history = []
        self.total_cost_history = []
        self.cluster_costs_history = {}
        
        self.get_logger().info(f"🔄 Starte neuen Durchlauf ({self.current_run}). Setze k=1.")
        
        # Startet den Loop wieder nach 1 Sekunde Pause
        self.timer = self.create_timer(1.0, self.evaluate_next_k)

    def evaluate_statistics_and_shutdown(self):
        """Wertet die Daten aus, schreibt die CSV und beendet ROS."""
        costs = [res['cost'] for res in self.experiment_results]
        ks = [res['k'] for res in self.experiment_results]
        p = self.optimizer.params

        self.get_logger().info("\n=========================================")
        self.get_logger().info("🏆 EXPERIMENT ABGESCHLOSSEN 🏆")
        self.get_logger().info(f"Gesamt-Durchläufe: {len(self.experiment_results)}")
        self.get_logger().info(f"Kosten (J_total): Durchschnitt = {np.mean(costs):.2f}, StdAbw = {np.std(costs):.2f}")
        self.get_logger().info(f"Gewähltes k:      Durchschnitt = {np.mean(ks):.2f}, StdAbw = {np.std(ks):.2f}")
        self.get_logger().info("=========================================\n")

        # CSV Export (Append-Modus für mehrere Messobjekte)
        # KORREKTUR: Pfad als Parameter, damit parallele Läufe getrennte Dateien nutzen
        file_path = self.csv_path
        os.makedirs(os.path.dirname(file_path) or '.', exist_ok=True)
        
        # NEU: Prüfen, ob die Datei schon existiert
        file_exists = os.path.isfile(file_path)
        
        try:
            # NEU: mode='a' (append) hängt Daten unten an, statt sie zu überschreiben
            with open(file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                
                # NEU: Spaltenköpfe NUR schreiben, wenn die Datei neu erstellt wird
                if not file_exists:
                    writer.writerow(['Task_ID', 'Messobjekt', 'Run_ID', 'Status', 'Gewaehltes_k',
                                     'Finale_Kosten_J', 'cost_crlb', 'cost_move', 'cost_obs', 'cost_inter',
                                     'k_Verlauf', 'Kosten_Verlauf', 'Rechenzeit_Verlauf_s', 'Rechenzeit_gesamt_s',
                                     'w_move', 'w_obs', 'd_safe', 'd_crash', 'robot_types', 'N_Roboter',
                                     'N_Drohnenposen', 'Gruppengroessen', 'Formationen'])
                
                for res in self.experiment_results:
                    # Hier müssen die Keys exakt so heißen wie oben im Dictionary!
                    writer.writerow([
                        res['task_id'],
                        res['target'], 
                        res['run'], 
                        res['status'],
                        res['k'], 
                        res['cost'], 
                        res['cost_crlb'], 
                        res['cost_move'], 
                        res['cost_obs'], 
                        res['cost_inter'],
                        json.dumps(res['k_history']), 
                        json.dumps(res['cost_history']),
                        json.dumps(res['runtime_history']),
                        res['runtime_total'],
                        self.get_parameter('w_move').value,
                        p.get('w_obs'),
                        p.get('d_safe'),
                        p.get('d_crash'),
                        ''.join(p.get('robot_types', [])),
                        len(self.start_robot_poses) // 2,
                        res['n_drones'],
                        json.dumps(res['group_sizes']),
                        json.dumps(res['formations']),
                    ])

            self.get_logger().info(f"💾 CSV erfolgreich gespeichert/erweitert unter: {file_path}")
        except Exception as e:
            self.get_logger().error(f"Fehler beim Speichern der CSV: {e}")

        if rclpy.ok():
            rclpy.shutdown()

        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = OptimizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Dieser Block wird aufgerufen, wenn du Strg + C drückst!
        node.get_logger().info("\n⚠️ Experiment manuell abgebrochen (Strg+C)!")
        
        # Prüfen, ob wir im Batch-Modus sind und schon Daten gesammelt haben
        if hasattr(node, 'enable_batch_evaluation') and node.enable_batch_evaluation:
            if len(node.experiment_results) > 0:
                node.get_logger().info(f"Speichere die bisherigen {len(node.experiment_results)} Durchläufe ab...")
                # Führt die Auswertung durch, schreibt die CSV und beendet sich (sys.exit)
                node.evaluate_statistics_and_shutdown()
            else:
                node.get_logger().info("Noch kein Durchlauf vollständig beendet. Beende ohne Speichern.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()