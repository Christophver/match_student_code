import numpy as np
import math

from skimage.draw import line # Wichtig für den Bresenham!

# ==========================================================================
# MESSMODELL (Absolute Multiline): sigma_pos = 0,2 µm + 0,3 µm/m * d
# Abstände d in Metern, Positionsvarianzen und A-Kriterium in µm².
# ==========================================================================
SIGMA_POS_0_UM = 0.2        # µm
ALPHA_POS_UM_PER_M = 0.3    # µm/m
D_MAX_M = 30.0              # Messreichweite in m

# Strafwerte (vgl. Kapitel 5)
C_BASE = 1e8                # nicht beobachtbare Drohnenpose
C_STEP = 1e7                # je ungültigem Tracker
C_NUM = 1e9                 # zu kleine Flotte / numerischer Fehler
P_MAP = 1e7                 # Roboter außerhalb der Karte
P_CRASH = 1e6               # Kollision


class ESDFMapVectorized:
    def __init__(self, esdf_matrix, pixel_to_meter, origin_x=0.0, origin_y=0.0):
        self.grid = esdf_matrix.astype(np.float32)
        self.resolution = float(pixel_to_meter)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.height, self.width = self.grid.shape

    def get_distance_and_angle_batch(self, wx_array, wy_array):
        """
        Berechnet Distanz und Winkel für ein NumPy-Array von Positionen gleichzeitig.
        :param wx_array: 1D NumPy-Array der X-Koordinaten aller Roboter/Partikel
        :param wy_array: 1D NumPy-Array der Y-Koordinaten aller Roboter/Partikel
        """
        # 1. Transformation in den Pixelraum (Vektorisiert)
        # KORREKTUR: Die ESDF-Werte gelten für die Zellmittelpunkte. Der Mittelpunkt
        # der Zelle (i, j) liegt bei origin + (j + 0,5) * resolution. Ohne den
        # Abzug von 0,5 war die Interpolation um eine halbe Zelle verschoben.
        px = (wx_array - self.origin_x) / self.resolution - 0.5
        py = (wy_array - self.origin_y) / self.resolution - 0.5

        x0 = np.floor(px).astype(np.int32)
        y0 = np.floor(py).astype(np.int32)
        x1 = x0 + 1
        y1 = y0 + 1

        # Randschutz-Maske erstellen (Gibt True aus, wenn Partikel innerhalb der Grenzen liegt)
        valid_mask = (x0 >= 0) & (x1 < self.width) & (y0 >= 0) & (y1 < self.height)

        # Sichere Indizes für die Matrix-Abfrage (ungültige Werte temporär auf 0 setzen, um IndexErrors zu vermeiden)
        x0_safe = np.clip(x0, 0, self.width - 1)
        x1_safe = np.clip(x1, 0, self.width - 1)
        y0_safe = np.clip(y0, 0, self.height - 1)
        y1_safe = np.clip(y1, 0, self.height - 1)

        # Lokale Gewichte
        u = px - x0
        v = py - y0

        # Werte aus dem Grid auslesen
        d00 = self.grid[y0_safe, x0_safe]
        d10 = self.grid[y0_safe, x1_safe]
        d01 = self.grid[y1_safe, x0_safe]
        d11 = self.grid[y1_safe, x1_safe]

        # Bilineare Interpolation (Vektorisiert über alle Elemente)
        dist_pixel = (1 - u) * (1 - v) * d00 + u * (1 - v) * d10 + (1 - u) * v * d01 + u * v * d11
        distance_meters = dist_pixel * self.resolution

        # Analytischer Gradient
        grad_u = (1 - v) * (d10 - d00) + v * (d11 - d01)
        grad_v = (1 - u) * (d01 - d00) + u * (d11 - d10)

        # Richtung zur Wand bestimmen
        grad_x = -grad_u / self.resolution
        grad_y = -grad_v / self.resolution
        target_yaw = np.arctan2(grad_y, grad_x)

        # Ungültige Werte (außerhalb der Karte) mit Strafwerten überschreiben
        distance_meters = np.where(valid_mask, distance_meters, -1.0)
        target_yaw = np.where(valid_mask, target_yaw, 0.0)

        return distance_meters, target_yaw
    

class SwarmOptimizer:
    def __init__(self, params):
        self.params = params
        self.num_robots = 3 
        
        # Karten-Daten (werden von der Node injiziert, sobald sie eintreffen)
        self.esdf_map = None
        self.binary_grid = None
        self.map_resolution = 0.1
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        
        # Lade Schwellenwerte einmalig zur Optimierung
        # KORREKTUR: Rückfallwert an den Knoten angeglichen (dort 1,0 m)
        self.d_safe = self.params.get('d_safe', 1.0)
        self.d_crash = self.params.get('d_crash', 0.15)

        # NEU (Laufzeit): Zwischenspeicher für Sichtlinienprüfungen. Die Prüfung hängt
        # nur von der Rasterzelle des Trackers und der Rasterzelle der Drohne ab
        # (2,5D-Karte, Höhe spielt keine Rolle). Je Drohnenzelle speichert ein
        # int8-Raster das Ergebnis (-1 unbekannt, 0 frei, 1 verdeckt). Das Ergebnis
        # ist identisch mit der direkten Prüfung, wird aber je Zellenpaar nur einmal
        # berechnet und über alle Gruppen und Gruppenanzahlen wiederverwendet.
        self._los_cache = {}
        self._los_cache_grid_id = None

        # Kostenzusammensetzung der besten Formation (für die CSV-Auswertung)
        self.best_crlb = 0.0
        self.best_obs = 0.0
        self.best_obs_raw = 0.0
        self.best_inter = 0.0

    def run_pso(self, drone_cluster, start_robot_poses):
        cluster_center = np.mean(drone_cluster, axis=0) 
        
        
        # 1. Anzahl der Roboter dynamisch auslesen (x,y pro Roboter)
        self.num_robots = len(start_robot_poses) // 2
        dimensions = self.num_robots * 2
        
        # 2. Dynamische Partikelanzahl: N = 10 * D (mindestens 30)
        num_particles = max(30, 10 * dimensions)
        
        particles = np.random.uniform(-15, 15, (num_particles, dimensions))
        
        # Streuung der Partikel um das Cluster-Zentrum
        for i in range(self.num_robots):
            particles[:, i*2] += cluster_center[0]     
            particles[:, i*2+1] += cluster_center[1]   

        # NEU: Partikel zwingend auf der Karte halten (verhindert Out-of-Bounds)
        grid_height, grid_width = self.binary_grid.shape  # <-- NEU: Hier holen wir die Größe!
        
        map_min_x = self.map_origin_x + 1.0
        map_max_x = self.map_origin_x + (grid_width * self.map_resolution) - 1.0
        map_min_y = self.map_origin_y + 1.0
        map_max_y = self.map_origin_y + (grid_height * self.map_resolution) - 1.0

        for i in range(self.num_robots):
            particles[:, i*2] = np.clip(particles[:, i*2], map_min_x, map_max_x)
            particles[:, i*2+1] = np.clip(particles[:, i*2+1], map_min_y, map_max_y)
            
        best_formation = None
        
        # ==========================================
        # 3. SCHWARM-BEWEGUNG MIT ABBRUCHKRITERIEN
        # ==========================================
        # Initialisierung der Geschwindigkeiten und Partikel-Erinnerungen
        velocities = np.zeros((num_particles, dimensions))
        personal_best_positions = particles.copy()
        personal_best_costs = np.full(num_particles, float('inf'))
        
        global_best_position = None
        best_cost = float('inf')
        
        stagnation_counter = 0
        # Werte dynamisch aus den übergebenen Parametern holen
        # KORREKTUR: Rückfallwert von epsilon an den Knoten angeglichen (1e-4)
        patience = self.params.get('t_patience', 15)
        min_improvement = self.params.get('epsilon', 1e-4)
        
        # PSO Hyperparameter dynamisch laden
        max_iterations = self.params.get('max_iterations', 50)
        w = self.params.get('inertia_weight', 0.5)  
        c1 = self.params.get('c1', 1.5)             
        c2 = self.params.get('c2', 1.5)             
        

        stop_reason = 'max_iter'   # Protokoll für die Auswertung der Abbruchkriterien
        for iteration in range(max_iterations):
            improved = False
            
            # 1. Fitness evaluieren und pBest/gBest updaten
            for i in range(num_particles):
                
                cost, curr_crlb, curr_obs, curr_inter, curr_obs_raw = self._calculate_cost(particles[i], drone_cluster)
                
                # Update Personal Best (lokales Optimum des Partikels)
                if cost < personal_best_costs[i]:
                    personal_best_costs[i] = cost
                    personal_best_positions[i] = particles[i].copy()
                
                # Update Global Best (Schwrm-Optimum)
                if cost < best_cost - min_improvement:
                    best_cost = cost
                    global_best_position = particles[i].copy()
                    improved = True
                    
                    # ÄNDERUNG 2: Speichere die Kosten-Zusammensetzung dieses Rekords
                    self.best_crlb = curr_crlb
                    self.best_obs = curr_obs
                    self.best_obs_raw = curr_obs_raw
                    self.best_inter = curr_inter
                    

            # ==========================================
            # NEUER SICHERHEITSCHECK falls alle Partikel im Hindernis landen
            # ==========================================
            if global_best_position is None:
                # Der komplette Schwarm steckt im Hindernis fest. 
                # Sofortiger Abbruch, Melde "Unmöglich" an den Hauptknoten!
                print(f"PSO_STOP iterationen={iteration + 1} grund=kein_bestwert "
                      f"partikel={num_particles}", flush=True)
                return None, float('inf')
            

            # --- START PARTIKEL UPDATE ---
            # Zufallsmatrizen r1 und r2 (stochastische Komponente)
            r1 = np.random.rand(num_particles, dimensions)
            r2 = np.random.rand(num_particles, dimensions)
            
            # Geschwindigkeiten berechnen (Kanonische PSO Gleichung)
            velocities = (w * velocities + 
                          c1 * r1 * (personal_best_positions - particles) + 
                          c2 * r2 * (global_best_position - particles))
            
            # Maximale Geschwindigkeit begrenzen (verhindert Schwarm-Explosion)
            v_max = 2.0 
            velocities = np.clip(velocities, -v_max, v_max)
            
            # Positionen aktualisieren
            particles = particles + velocities
            # --- ENDE PARTIKEL UPDATE ---
            
            # Stagnation hochzählen oder zurücksetzen
            if not improved:
                stagnation_counter += 1
            else:
                stagnation_counter = 0 
                
            # Early Stopping
            if stagnation_counter >= patience:
                stop_reason = 'stagnation'
                break
        
        # Eine Zeile je PSO-Aufruf (Gruppe) im Protokoll des Laufs; nur Ausgabe,
        # ohne Einfluss auf das Ergebnis
        print(f"PSO_STOP iterationen={iteration + 1} grund={stop_reason} "
              f"partikel={num_particles} kosten={best_cost:.6g}", flush=True)
        return global_best_position, best_cost
    
    def _calculate_cost(self, formation, drone_cluster, use_minimax=False):
        crlb_values = []
        
        for drone_pos in drone_cluster:
            crlb_val = self._calculate_crlb_3d_from_ground(formation, drone_pos)
            crlb_values.append(crlb_val)
            
        if use_minimax:
            base_cost = max(crlb_values) if crlb_values else float('inf')
        else:
            base_cost = sum(crlb_values)
            
        # HIER WIRD DAS GEWICHT ANGEWENDET
        # 1. w_obs aus den Parametern laden (Standardwert 1.0, falls nichts übergeben wird)
        w_obs = self.params.get('w_obs', 1.0)
        
        # 2. Rohe Penalty berechnen
        c_obs_raw = self._calculate_obstacle_penalty(formation)
        
        # 3. Mit dem Gewicht multiplizieren!
        c_obs = w_obs * c_obs_raw  
        
        c_inter = self._calculate_inter_robot_penalty(formation)
        
        # Gesamtkosten zusammensetzen
        total_cost = base_cost + c_obs + c_inter
        
        # Reihenfolge: Gesamtsumme, CRLB, Hindernisse (gewichtet), Inter-Roboter,
        # Hindernisse (ungewichtet, für die Kollisionsprüfung der äußeren Ebene)
        return total_cost, base_cost, c_obs, c_inter, c_obs_raw

    def _calculate_crlb_3d_from_ground(self, formation, drone_pose):
        J = np.zeros((6, 6))
        
        # DEFINITION DEINER FLOTTE: 
        # Hier legst du fest, welche Roboter du ins Feld schickst.
        # Beispiel: 4 Roboter, aber insgesamt 6 Tracker (2x Typ A, 2x Typ B)
        # Die Länge dieser Liste muss exakt self.num_robots entsprechen!
        # 1. Parameter dynamisch laden (Fallback auf 6 Roboter, falls YAML fehlt)
        robot_types = self.params.get('robot_types', ['A', 'A', 'A', 'A', 'B', 'B'])
        
        # 2. BULLETPROOF-SICHERHEIT: Liste immer exakt an die Startposen anpassen!
        if len(robot_types) < self.num_robots:
            # Liste ist zu kurz? Mit Typ 'A' auffüllen, bis es passt
            robot_types = list(robot_types) + ['A'] * (self.num_robots - len(robot_types))
        elif len(robot_types) > self.num_robots:
            # Liste ist zu lang? Abschneiden
            robot_types = robot_types[:self.num_robots]
        
        # Liste für alle berechneten Tracker-Positionen
        tracker_positions = []

        # 1. Tracker-Positionen aus den Roboter-Positionen extrahieren
        for i in range(self.num_robots):
            rx, ry = formation[i*2], formation[i*2+1]
            r_type = robot_types[i]
            
            if r_type == 'A':
                # Typ A: 1 Tracker zentral auf dem Roboter
                tracker_positions.append([rx, ry, 0.0])
                
            elif r_type == 'B':
                # Typ B: 2 Tracker mit 20cm Abstand, orthogonal zur Sichtlinie
                dx = drone_pose[0] - rx
                dy = drone_pose[1] - ry
                
                # Distanz am Boden
                dist_ground = math.hypot(dx, dy)
                
                if dist_ground > 0.001:
                    # Normalisierter Richtungsvektor
                    ux = dx / dist_ground
                    uy = dy / dist_ground
                    
                    # Orthogonalvektor (90 Grad gedreht)
                    nx = -uy
                    ny = ux
                    
                    # Basislinie: 10cm in jede Richtung
                    offset = 0.1 
                    
                    t1_x = rx + offset * nx
                    t1_y = ry + offset * ny
                    
                    t2_x = rx - offset * nx
                    t2_y = ry - offset * ny
                    
                    # KORREKTUR: erster Tracker verwendete t2_y statt t1_y
                    tracker_positions.append([t1_x, t1_y, 0.0])
                    tracker_positions.append([t2_x, t2_y, 0.0])
                else:
                    # Fallback, falls die Drohne exakt senkrecht drüber ist
                    tracker_positions.append([rx + 0.1, ry, 0.0])
                    tracker_positions.append([rx - 0.1, ry, 0.0])

        # Wenn wir durch Typ A/B Kombinationen weniger als 6 Tracker haben, 
        # ist die 6D-Matrix mathematisch nicht voll rangfähig (singulär).
        # KORREKTUR: SwarmOptimizer besitzt kein get_logger() (kein ROS-Knoten).
        # Der Aufruf führte zum Absturz. Die Flottengröße prüft jetzt der Knoten.
        if len(tracker_positions) < 6:
            return float(C_NUM)

        # 2. CRLB für JEDEN TRACKER berechnen (nicht mehr pro Roboter!)
        # NEU (Laufzeit): vektorisierte Berechnung über alle Tracker. Das Ergebnis
        # entspricht der bisherigen Schleife bis auf Rundungsunterschiede (~1e-15).
        T = np.asarray(tracker_positions, dtype=float)
        D = np.asarray(drone_pose[:3], dtype=float) - T          # Vektoren Tracker -> Drohne
        d_3d = np.sqrt((D * D).sum(axis=1))

        in_range = (d_3d <= D_MAX_M) & (d_3d != 0)
        valid = np.zeros(len(T), dtype=bool)
        for j in np.flatnonzero(in_range):
            # LoS-Check (Hindernisprüfung) mit Zwischenspeicher
            valid[j] = not self._is_los_blocked_cached(T[j], drone_pose)
        blocked_count = int(len(T) - np.count_nonzero(valid))

        J = np.zeros((6, 6))
        if np.any(valid):
            dv = d_3d[valid]
            U = D[valid] / dv[:, None]                           # Einheitsvektoren
            # --- POSITIONS-BLOCK --- sigma_pos in µm (d in m)
            w_pos = 1.0 / (SIGMA_POS_0_UM + ALPHA_POS_UM_PER_M * dv) ** 2
            J[0:3, 0:3] = (U * w_pos[:, None]).T @ U               # symmetrisch per Konstruktion
            # --- WINKEL-BLOCK ---
            inv_var_ang = np.sum(1.0 / (0.05 + 0.01 * dv) ** 2)
            J[3, 3] = J[4, 4] = J[5, 5] = inv_var_ang

        # --- BULLETPROOF MATRIX INVERTIERUNG ---
        try:
            # 1. Determinante prüfen (Toleranz etwas strikter setzen)
            det = np.linalg.det(J)
            if det < 1e-5 or math.isnan(det):
                return float(C_BASE) + (blocked_count * C_STEP)
                
            # 2. Invertieren und Spur berechnen
            J_inv = np.linalg.inv(J)
            crlb_trace = np.trace(J_inv)
            
            # 3. DER WICHTIGSTE CHECK: CRLB darf NIEMALS negativ oder NaN sein!
            # Fängt den -10^17 Glitch ab und bestraft die fehlerhafte Formation
            if crlb_trace <= 0 or math.isnan(crlb_trace) or math.isinf(crlb_trace):
                return float(C_NUM)
                
            return crlb_trace
            
        except np.linalg.LinAlgError:
            # Fängt ab, falls NumPy die Matrix intern als komplett unlösbar einstuft
            return float(C_NUM)
            
        
    
    
    def _is_los_blocked_cached(self, start_pos, end_pos):
        """Wie _is_los_blocked, aber mit Zwischenspeicher je Zellenpaar (identisches Ergebnis)."""
        # Zwischenspeicher verwerfen, falls eine neue Karte gesetzt wurde
        if self._los_cache_grid_id != id(self.binary_grid):
            self._los_cache = {}
            self._los_cache_grid_id = id(self.binary_grid)

        height, width = self.binary_grid.shape
        x0 = math.floor((start_pos[0] - self.map_origin_x) / self.map_resolution)
        y0 = math.floor((start_pos[1] - self.map_origin_y) / self.map_resolution)
        x1 = math.floor((end_pos[0] - self.map_origin_x) / self.map_resolution)
        y1 = math.floor((end_pos[1] - self.map_origin_y) / self.map_resolution)

        # Tracker außerhalb der Karte: ohne Zwischenspeicher prüfen
        if not (0 <= x0 < width and 0 <= y0 < height):
            return self._is_los_blocked(start_pos, end_pos)

        key = (x1, y1)
        grid = self._los_cache.get(key)
        if grid is None:
            grid = np.full((height, width), -1, dtype=np.int8)
            self._los_cache[key] = grid
        val = grid[y0, x0]
        if val < 0:
            val = 1 if self._is_los_blocked(start_pos, end_pos) else 0
            grid[y0, x0] = val
        return bool(val)

    def _is_los_blocked(self, start_pos, end_pos):
        """
        Prüft die Sichtlinie. Nutzt das in C-optimierte skimage.draw.line
        für maximale Vektorisierungs-Geschwindigkeit in Python.
        """
        # Weltkoordinaten in Pixel umrechnen
        # KORREKTUR: floor statt int, damit negative Koordinaten (außerhalb der
        # Karte) in die richtige Zelle fallen (int rundet in Richtung null).
        x0 = math.floor((start_pos[0] - self.map_origin_x) / self.map_resolution)
        y0 = math.floor((start_pos[1] - self.map_origin_y) / self.map_resolution)
        x1 = math.floor((end_pos[0] - self.map_origin_x) / self.map_resolution)
        y1 = math.floor((end_pos[1] - self.map_origin_y) / self.map_resolution)

        # skimage nutzt (Zeile, Spalte), also (y, x)
        # Dieser Aufruf läuft komplett in C ab!
        rr, cc = line(y0, x0, y1, x1)

        height, width = self.binary_grid.shape

        # Array-Grenzen sichern (Boolean Masking ist extrem schnell)
        valid = (rr >= 0) & (rr < height) & (cc >= 0) & (cc < width)
        rr, cc = rr[valid], cc[valid]

        # Die ersten 3 Pixel ignorieren (Zellen am Tracker, d. h. am Startpunkt der
        # Linie; verhindert, dass ein Tracker direkt an einer Wand sich selbst verdeckt)
        if len(rr) > 3:
            rr = rr[3:]
            cc = cc[3:]
        elif len(rr) == 0:
            return False # Leeres Array = keine Wand

        # Vektorisierter Check auf Wände (läuft ebenfalls in C ab)
        return bool(np.any(self.binary_grid[rr, cc] > 0))

    def _calculate_obstacle_penalty(self, formation):
        penalty = 0.0
        
        # 1. Formation [x1, y1, x2, y2, ...] in zwei Arrays splitten
        rx_array = formation[0::2]
        ry_array = formation[1::2]
        
        # 2. Vektorisierte Abfrage an dein ESDF!
        distances, _ = self.esdf_map.get_distance_and_angle_batch(rx_array, ry_array)
        
        # 3. Penalty berechnen
        for d_obs in distances:
            if d_obs < 0.0:
                penalty += P_MAP    # Roboter außerhalb der Karte
            elif d_obs <= self.d_crash:
                penalty += P_CRASH  # Kollision mit Hindernis
            elif d_obs < self.d_safe:
                penalty += ((1.0 / d_obs) - (1.0 / self.d_safe)) ** 2
        return penalty

    def _calculate_inter_robot_penalty(self, formation):
        penalty = 0.0
        
        # Parameter analog zur Hindernisvermeidung
        d_min = 1.5    # Sicherheitsabstand (Repulsive Kraft beginnt)
        d_crash = 0.6  # 2 x 0,3 m Roboterradius -> physische Kollision
        
        # Dynamische Schleife zur Kollisionsprüfung aller Roboter-Paare
        for i in range(self.num_robots):
            for j in range(i+1, self.num_robots):
                rx1, ry1 = formation[i*2], formation[i*2+1]
                rx2, ry2 = formation[j*2], formation[j*2+1]
                
                dist = math.hypot(rx1-rx2, ry1-ry2)
                
                if dist <= d_crash:
                    # Physisch unmöglich -> Kollisionsstrafe
                    penalty += P_CRASH
                elif dist < d_min:
                    # Exponentieller Anstieg der Abstoßungskraft, je näher sie kommen
                    penalty += ((1.0 / dist) - (1.0 / d_min)) ** 2
                    
        return penalty