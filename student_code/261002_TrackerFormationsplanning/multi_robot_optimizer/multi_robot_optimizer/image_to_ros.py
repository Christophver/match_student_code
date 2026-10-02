import cv2
import numpy as np
import math
import os
import time

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from sklearn.cluster import KMeans
from geometry_msgs.msg import PoseArray, Pose
import scipy.ndimage as ndimage  

# ==========================================
# KONFIGURATION (Globale Variablen)
# ==========================================
PIXEL_TO_METER = 0.1              # Skalierung: 1 Pixel = 10 cm in der echten Welt
BUILDING_HEIGHT = 10.0            # Angenommene Standardhöhe aller Gebäude in Metern

# NEU: Konstanten für den Streifenlichtprojektor (Structured Light Scanner)
STANDOFF_DISTANCE_M = 2.5         # Fester Arbeitsabstand der Drohne zur Wand
TARGET_RESOLUTION_M = 2.5         # Gewünschter Abstand der Dummy-Drohnen untereinander (Adaptive Sampling)

selected_target_idx = 0
contours = []
image_display = None

def mouse_callback(event, x, y, flags, param):
    """Reagiert auf Mausklicks im Safety-Check-Fenster."""
    global selected_target_idx, contours, image_display
    if event == cv2.EVENT_LBUTTONDOWN:
        for idx, cnt in enumerate(contours):
            if cv2.pointPolygonTest(cnt, (x, y), False) >= 0:
                selected_target_idx = idx
                print(f"--> Neues Messobjekt manuell ausgewählt: Index {idx}")
                # draw_and_show() # Wieder einkommentieren, wenn die Funktion existiert
                break

def generate_hardcoded_scenario(mode):
    """Generiert die 4 Masterarbeits-Welten direkt im RAM"""
    img = np.ones((200, 200), dtype=np.uint8) * 255 # Weiße Leinwand (20x20 Meter)

    if mode == 1: # Simple
        cv2.rectangle(img, (80, 80), (120, 120), 0, -1)
        
    elif mode == 2: # Simple + Wand
        cv2.rectangle(img, (80, 80), (120, 120), 0, -1)
        cv2.rectangle(img, (40, 150), (160, 160), 0, -1)
        
    elif mode == 3: # U-Profil (Breit)
        cv2.rectangle(img, (50, 60), (70, 140), 0, -1)
        cv2.rectangle(img, (50, 120), (150, 140), 0, -1)
        cv2.rectangle(img, (130, 60), (150, 140), 0, -1)
        
    elif mode == 4: # U-Profil im Käfig
        cv2.rectangle(img, (50, 60), (70, 140), 0, -1)
        cv2.rectangle(img, (50, 120), (150, 140), 0, -1)
        cv2.rectangle(img, (130, 60), (150, 140), 0, -1)
        cv2.rectangle(img, (30, 180), (170, 190), 0, -1) 
        cv2.line(img, (10, 20), (40, 50), 0, thickness=8)
        cv2.line(img, (190, 20), (160, 50), 0, thickness=8)

    return img

def draw_and_show():
    """Zeichnet die Gebäude und markiert das ausgewählte Messobjekt grün."""
    global image_display, contours, selected_target_idx
    
    # Sicherheitscheck, falls das Bild noch nicht geladen ist
    if image_display is None:
        return
        
    # Eine Kopie des Originalbildes erstellen, um darauf zu zeichnen
    temp_img = image_display.copy()
    
    # 1. Alle gefundenen Gebäude/Hindernisse ROT umranden (Dicke: 2)
    cv2.drawContours(temp_img, contours, -1, (0, 0, 255), 2)
    
    # 2. Das aktuell ausgewählte Messobjekt GRÜN und ausgefüllt (FILLED) zeichnen
    if len(contours) > 0 and selected_target_idx < len(contours):
        cv2.drawContours(temp_img, [contours[selected_target_idx]], -1, (0, 255, 0), cv2.FILLED)
        
    # Bild im Fenster aktualisieren
    cv2.imshow("Safety Check - Klicke zur Korrektur, druecke ENTER zum Bestaetigen", temp_img)

# ==========================================
# NEU: Eingabe von Drohnenposen aus einer Datei
# ==========================================
# Format (CSV, eine Pose je Zeile, Einheiten m bzw. rad):
#     x, y, z[, yaw]
# - Koordinaten im Kartenrahmen "map": Ursprung in der Bildmitte, x nach rechts,
#   y nach oben (wie die erzeugten Posen und die Karte).
# - yaw ist optional. Fehlt er, richtet sich die Drohne zum Mittelpunkt des
#   Messobjekts aus. Der Gierwinkel beeinflusst die Optimierung nicht.
# - Trennzeichen Komma oder Semikolon; Zeilen mit '#' und eine Kopfzeile
#   (z. B. "x,y,z,yaw") werden übersprungen.
def lade_drohnenposen(pfad, logger):
    posen = []
    with open(pfad, 'r', encoding='utf-8') as fh:
        for nr, zeile in enumerate(fh, start=1):
            zeile = zeile.split('#', 1)[0].strip()
            if not zeile:
                continue
            teile = [t.strip() for t in zeile.replace(';', ',').split(',') if t.strip()]
            try:
                werte = [float(t) for t in teile]
            except ValueError:
                if nr == 1:
                    continue          # Kopfzeile
                logger.warn(f"Zeile {nr} in {pfad} nicht lesbar und übersprungen: {zeile}")
                continue
            if len(werte) not in (3, 4):
                logger.warn(f"Zeile {nr}: 3 oder 4 Werte erwartet (x, y, z[, yaw]), übersprungen.")
                continue
            posen.append(werte)
    return posen


def pruefe_drohnenposen(posen, occupied, origin_x, origin_y, h, w, tcx, tcy, logger):
    """Verwirft Posen außerhalb der Karte, in Hindernissen oder mit z <= 0 und
    ergänzt fehlende Gierwinkel (Ausrichtung zum Messobjekt)."""
    gueltig = []
    for p in posen:
        x, y, z = p[0], p[1], p[2]
        ci = int(math.floor((y - origin_y) / PIXEL_TO_METER))
        cj = int(math.floor((x - origin_x) / PIXEL_TO_METER))
        if not (0 <= ci < h and 0 <= cj < w):
            logger.warn(f"Pose ({x:.2f}, {y:.2f}, {z:.2f}) liegt außerhalb der Karte und entfällt.")
            continue
        if occupied[ci, cj]:
            logger.warn(f"Pose ({x:.2f}, {y:.2f}, {z:.2f}) liegt in einem Hindernis und entfällt.")
            continue
        if z <= 0.0:
            logger.warn(f"Pose ({x:.2f}, {y:.2f}, {z:.2f}) hat keine positive Höhe und entfällt.")
            continue
        yaw = p[3] if len(p) == 4 else math.atan2(tcy - y, tcx - x)
        gueltig.append([x, y, z, yaw])
    return gueltig


def main(args=None):
    # ==========================================
    # 1. ROS 2 Node initialisieren (NUR EINMAL!)
    # ==========================================
    rclpy.init(args=args)
    node = rclpy.create_node('map_publisher_node')

    # ==========================================
    # 2. PARAMETER AUSLESEN
    # ==========================================
    node.declare_parameter('scenario', 0)
    node.declare_parameter('custom_image_path', '/home/vboxuser/map_ws/Mein_Luftbild.png')
    # NEU: Herkunft der Drohnenposen
    #   'generiert' – Posen wie bisher an der Iso-Kontur erzeugen (Standard)
    #   'datei'     – Posen aus der CSV-Datei 'drohnen_datei' laden
    #   'extern'    – keine Posen senden; ein anderer Knoten sendet sie auf
    #                 /drone_targets (geometry_msgs/PoseArray, frame "map")
    node.declare_parameter('drohnen_quelle', 'generiert')
    node.declare_parameter('drohnen_datei', '')
    # NEU: optional die verwendeten Posen als CSV speichern (Vorlage/Kontrolle)
    node.declare_parameter('drohnen_export', '')

    scenario_mode = node.get_parameter('scenario').value
    img_path = node.get_parameter('custom_image_path').value
    drohnen_quelle = str(node.get_parameter('drohnen_quelle').value).strip().lower()
    drohnen_datei = os.path.expanduser(str(node.get_parameter('drohnen_datei').value))
    drohnen_export = os.path.expanduser(str(node.get_parameter('drohnen_export').value))
    if drohnen_quelle not in ('generiert', 'datei', 'extern'):
        node.get_logger().error(f"Unbekannte drohnen_quelle '{drohnen_quelle}' "
                                "(erlaubt: generiert, datei, extern).")
        return
    if drohnen_quelle == 'datei' and not drohnen_datei:
        node.get_logger().error("drohnen_quelle 'datei' benötigt den Parameter drohnen_datei.")
        return

    # ==========================================
    # 3. BILDQUELLE WÄHLEN
    # ==========================================
    node.get_logger().info(f"Starte Image-Node im Szenario-Modus: {scenario_mode}")

    if scenario_mode == 0:
        # ---> Bisheriger Workflow (Pinta) <---
        node.get_logger().info(f"Lade Bild von: {img_path}")
        img = cv2.imread(img_path) # Direkt als BGR (Farbbild) laden!
        if img is None:
            node.get_logger().error(f"FEHLER: Konnte Pinta-Bild nicht laden: {img_path}")
            return
    else:
        # ---> Neuer Workflow (Masterarbeits-Welten) <---
        node.get_logger().info(f"Generiere hart-codiertes Szenario {scenario_mode}")
        gray_img = generate_hardcoded_scenario(scenario_mode)
        # In BGR (Farbe) konvertieren, damit die grünen Markierungen gleich funktionieren
        img = cv2.cvtColor(gray_img, cv2.COLOR_GRAY2BGR)

    # ==========================================
    # 4. KANTENERKENNUNG & MORPHOLOGIE
    # ==========================================
    global contours, image_display, selected_target_idx
    image_display = img.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 30, 100)
    
    kernel = np.ones((7,7), np.uint8)
    closing = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=3)
    
    found_contours, _ = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in found_contours if cv2.contourArea(c) > 100]

    if not contours:
        print("Keine Gebäude im Bild gefunden!")
        return

    # 5. Geometrische Heuristik
    h, w = img.shape[:2]
    center_x, center_y = w // 2, h // 2
    min_dist = float('inf')
    
    for idx, cnt in enumerate(contours):
        M = cv2.moments(cnt)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            dist = math.hypot(cx - center_x, cy - center_y)
            if dist < min_dist:
                min_dist = dist
                selected_target_idx = idx

    print(f"Heuristik: Gebäude {selected_target_idx} ist am nächsten zur Bildmitte.")

    # 6. Safety Check
    #cv2.namedWindow("Safety Check - Klicke zur Korrektur, druecke ENTER zum Bestaetigen", cv2.WINDOW_NORMAL)
    #cv2.resizeWindow("Safety Check - Klicke zur Korrektur, druecke ENTER zum Bestaetigen", 1280, 720)
    #cv2.setMouseCallback("Safety Check - Klicke zur Korrektur, druecke ENTER zum Bestaetigen", mouse_callback)

    #draw_and_show()

    #print("\n--- SAFETY CHECK ---")
    #print("Prüfe das Bildfenster! Ist das GRÜNE Objekt das korrekte Messobjekt?")
    #print("JA   -> Drücke die ENTER-Taste im Bildfenster")
    #print("NEIN -> Klicke mit der Maus auf das korrekte Gebäude, danach ENTER")

    #while True:
    #    key = cv2.waitKey(1) & 0xFF
    #    if key == 13 or key == 10: 
    #        break

    #cv2.destroyAllWindows()'

    # 6. Safety Check (FÜR AUTOMATISIERUNG DEAKTIVIERT)
    print("\n--- SAFETY CHECK ÜBERSPRUNGEN ---")
    print("Automatischer Modus aktiv. Warte 1,5 Sekunden für ROS 2 DDS Discovery...")
    time.sleep(1.5)

    

    # 7. Zielobjekt extrahieren (Für den Drohnen-Start)
    target_obstacles = []

    for idx, cnt in enumerate(contours):
        if idx == selected_target_idx:
            x, y, bw, bh = cv2.boundingRect(cnt)
            real_w = bw * PIXEL_TO_METER
            real_h = bh * PIXEL_TO_METER
            
            real_cx = ((x + bw/2.0) - center_x) * PIXEL_TO_METER
            real_cy = ((center_y) - (y + bh/2.0)) * PIXEL_TO_METER 
            
            obs_data = [round(real_cx, 2), round(real_cy, 2), BUILDING_HEIGHT/2.0, round(real_w, 2), round(real_h, 2), BUILDING_HEIGHT]
            target_obstacles.append(obs_data)

    # 8. Binäre Karte für ROS erstellen (OccupancyGrid)
    print("\nErstelle 2.5D OccupancyGrid...")
    obstacle_grid = np.zeros((h, w), dtype=np.uint8)
    cv2.drawContours(obstacle_grid, contours, -1, 255, thickness=cv2.FILLED)
    flipped_grid = np.flipud(obstacle_grid)

    flat_grid = flipped_grid.flatten()
    ros_grid = np.where(flat_grid > 0, 100, 0).astype(np.int8)

    # ==========================================
    # NEU: Adaptive Zielpunktgenerierung (Drohnenanzahl)
    # ==========================================
    target_contour = contours[selected_target_idx]
    perimeter_pixels = cv2.arcLength(target_contour, True)
    perimeter_meters = perimeter_pixels * PIXEL_TO_METER
    
    # Finale Drohnenanzahl (Dummys) berechnen
    num_drones = max(1, int(perimeter_meters / TARGET_RESOLUTION_M))

    if drohnen_quelle == 'generiert':
        print(f"--> Gebäudeumfang: {perimeter_meters:.2f} m")
        print(f"--> Adaptive Auflösung: {TARGET_RESOLUTION_M} m -> Generiere {num_drones} Dummy-Drohnen.")
    # ==========================================

    # Drohnen-Ring NUR um das Messobjekt berechnen
    print("Berechne maßgeschneiderte Drohnen-Positionen an der Iso-Kontur...")
    target_mask = np.zeros((h, w), dtype=np.uint8)
    cv2.drawContours(target_mask, [contours[selected_target_idx]], -1, 255, thickness=cv2.FILLED)
    flipped_target = np.flipud(target_mask)

    target_esdf = ndimage.distance_transform_edt(flipped_target == 0) * PIXEL_TO_METER
    
    # NEU: Iso-Kontur exakt bei 2.5m (STANDOFF_DISTANCE_M)
    tolerance = PIXEL_TO_METER / 2.0
    ring_mask = (target_esdf >= STANDOFF_DISTANCE_M - tolerance) & (target_esdf <= STANDOFF_DISTANCE_M + tolerance)

    origin_x = - (center_x * PIXEL_TO_METER)
    origin_y = - (center_y * PIXEL_TO_METER)

    valid_points = []
    for y in range(ring_mask.shape[0]):
        for x in range(ring_mask.shape[1]):
            if ring_mask[y, x]:
                # KORREKTUR: Zellmittelpunkt statt Zellecke (+0,5 Zellen)
                wx = origin_x + ((x + 0.5) * PIXEL_TO_METER)
                wy = origin_y + ((y + 0.5) * PIXEL_TO_METER)
                valid_points.append([wx, wy])

    valid_points = np.array(valid_points)
    drone_poses_list = []

    # Mittelpunkt des Messobjekts (für die Ausrichtung der Drohnen)
    x_box, y_box, bw, bh = cv2.boundingRect(contours[selected_target_idx])
    tcx = ((x_box + bw/2.0) - center_x) * PIXEL_TO_METER
    tcy = ((center_y) - (y_box + bh/2.0)) * PIXEL_TO_METER
    occupied = flipped_grid > 0

    if drohnen_quelle == 'datei':
        # NEU: Posen aus Datei laden und prüfen
        try:
            roh = lade_drohnenposen(drohnen_datei, node.get_logger())
        except OSError as e:
            node.get_logger().error(f"Drohnendatei nicht lesbar: {e}")
            return
        drone_poses_list = pruefe_drohnenposen(roh, occupied, origin_x, origin_y, h, w,
                                               tcx, tcy, node.get_logger())
        print(f"--> {len(drone_poses_list)} von {len(roh)} Drohnenposen aus {drohnen_datei} übernommen.")
        if not drone_poses_list:
            node.get_logger().error("Keine gültige Drohnenpose in der Datei.")
            return
    elif drohnen_quelle == 'extern':
        print("--> Drohnenposen kommen von einem anderen Knoten (/drone_targets).")

    # NEU: num_drones anstelle der festen 16 verwenden
    if drohnen_quelle != 'generiert':
        pass
    elif len(valid_points) >= num_drones:
        kmeans = KMeans(n_clusters=num_drones, random_state=42, n_init=10).fit(valid_points)
        z_levels = [1.25, 3.75, 6.25, 8.75]

        # KORREKTUR: Messsäulen, die in einem anderen Hindernis liegen, verwerfen.
        # Eine Drohne kann dort nicht fliegen, und jede Sichtlinie zu dieser Pose
        # wäre verdeckt. Die Pose wäre damit nie beobachtbar, und keine Gruppe mit
        # ihr könnte zulässig werden (Ursache für das Scheitern von Szenario 4).
        kept_centers = []
        for center in kmeans.cluster_centers_:
            ci = int(math.floor((center[1] - origin_y) / PIXEL_TO_METER))
            cj = int(math.floor((center[0] - origin_x) / PIXEL_TO_METER))
            if 0 <= ci < h and 0 <= cj < w and occupied[ci, cj]:
                print(f"--> Messsäule bei ({center[0]:.2f}, {center[1]:.2f}) liegt in einem Hindernis und entfällt.")
                continue
            kept_centers.append(center)

        for center in kept_centers:
            for z in z_levels:
                vx = tcx - center[0]
                vy = tcy - center[1]
                yaw = math.atan2(vy, vx)
                drone_poses_list.append([center[0], center[1], z, yaw])
    else:
        print(f"WARNUNG: Zielkontur zu klein für {num_drones} Drohnen!")


    # NEU: verwendete Posen optional als CSV speichern
    if drohnen_export and drone_poses_list:
        with open(drohnen_export, 'w', encoding='utf-8') as fh:
            fh.write("x,y,z,yaw\n")
            for dp in drone_poses_list:
                fh.write(f"{dp[0]:.4f},{dp[1]:.4f},{dp[2]:.4f},{dp[3]:.4f}\n")
        print(f"--> {len(drone_poses_list)} Drohnenposen gespeichert in {drohnen_export}")

    # 9. Publisher einrichten und Topics senden
    map_pub = node.create_publisher(OccupancyGrid, '/map', 10)
    drone_pub = node.create_publisher(PoseArray, '/drone_targets', 10)
    sende_drohnen = drohnen_quelle != 'extern'

    grid_msg = OccupancyGrid()
    grid_msg.header = Header(frame_id="map", stamp=node.get_clock().now().to_msg())
    grid_msg.info.resolution = PIXEL_TO_METER
    grid_msg.info.width = w
    grid_msg.info.height = h
    grid_msg.info.origin.position.x = origin_x
    grid_msg.info.origin.position.y = origin_y
    grid_msg.data = ros_grid.tolist()

    pose_array_msg = PoseArray()
    pose_array_msg.header = Header(frame_id="map", stamp=node.get_clock().now().to_msg())
    
    for dp in drone_poses_list:
        pose = Pose()
        pose.position.x = float(dp[0])
        pose.position.y = float(dp[1])
        pose.position.z = float(dp[2])
        pose.orientation.z = math.sin(dp[3] / 2.0)
        pose.orientation.w = math.cos(dp[3] / 2.0)
        pose_array_msg.poses.append(pose)

    print("Publiziere Map und Drohnen-Messpunkte...")
    # KORREKTUR: 20 statt 5 Wiederholungen (10 s statt 2,5 s). Unter Last startet
    # der Optimierungsknoten teils so langsam, dass er alle Nachrichten verpasste.
    for _ in range(20):
        map_pub.publish(grid_msg)
        if sende_drohnen:
            drone_pub.publish(pose_array_msg)
        time.sleep(0.5)

    print("✅ Übertragung erfolgreich abgeschlossen.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()