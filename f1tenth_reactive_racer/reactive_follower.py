#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class ReactiveFollowerNode(Node):
    def __init__(self):
        super().__init__('reactive_follower_node')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # ==========================================
        # ⚙️ PANEL DE CONTROL Y TUNING
        # ==========================================
        
        # --- 1. PARÁMETROS DE PERCEPCIÓN GEOMÉTRICA ---
        self.view_angle = 1.3           
        self.frontal_view_angle = 0.4    # Cono frontal estrecho para medir muros inminentes sin asustarse con las paredes laterales
        self.max_lidar_range = 30.0

        # --- 2. PARÁMETROS DEL DISPARITY EXTENDER Y SEGURIDAD ---
        self.car_radius = 0.4           # Radio físico + margen (0.25m evita roces protegiendo los pasillos estrechos del Escenario 2)
        self.disparity_threshold = 0.35  # Detecta bordes abruptos (ideal para el salto de 0.5m a 2.3m del Escenario 3)
        self.failsafe_dist = 0.35        # Distancia de emergencia si la visión falla

        # --- 3. PARÁMETROS DE VELOCIDAD ---
        self.max_speed = 5.0
        self.min_speed = 1.5
        self.braking_distance_vel = 6.0  # Empieza a frenar a 6 metros de la curva

        # --- 4. PARÁMETROS DE DIRECCIÓN (CONTROL PD) ---
        self.braking_distance_kp = 2.0
        self.Kp = 1.8
        self.k_vel = 2.8
        self.k_kp = 0.2 #7.0
        self.Kd = 0
        self.steering_attenuation = 0.5 #0.35
        self.max_steering_angle = 0.5

        # --- VARIABLES DE MEMORIA DEL CONTROLADOR ---
        self.prev_error = 0.0
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info('Piloto FTG + Disparity Extender PD iniciado.')

    def scan_callback(self, msg):
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        # ----------------------------------------------------
        # 1. PERCEPCIÓN Y LIMPIEZA
        # ----------------------------------------------------
        processed_ranges = []
        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r) or r == 0.0:
                processed_ranges.append(self.max_lidar_range)
            else:
                processed_ranges.append(r)

        start_idx = int((-self.view_angle - angle_min) / angle_inc)
        end_idx = int((self.view_angle - angle_min) / angle_inc)
        start_idx = max(0, start_idx)
        end_idx = min(len(processed_ranges) - 1, end_idx)
        # Cegar lo que está fuera del campo de visión frontal estableciendo a 0.0
        for i in range(len(processed_ranges)):
            if i < start_idx or i > end_idx:
                processed_ranges[i] = 0.0

        # ----------------------------------------------------
        # 2. DISPARITY EXTENDER (Ensanchar bordes de obstáculos)
        # ----------------------------------------------------
        disparities = []
        for i in range(start_idx, end_idx):
            if abs(processed_ranges[i] - processed_ranges[i+1]) > self.disparity_threshold:
                disparities.append(i)

        for i in disparities:
            dist_1 = processed_ranges[i]
            dist_2 = processed_ranges[i+1]
            
            if dist_1 < dist_2:
                # El punto de la derecha está más cerca, ensanchamos hacia la izquierda
                extend_angle = math.atan2(self.car_radius, dist_1)
                extend_idx = int(extend_angle / angle_inc)
                for j in range(i + 1, min(end_idx + 1, i + 1 + extend_idx)):
                    processed_ranges[j] = min(processed_ranges[j], dist_1)
            else:
                # El punto de la izquierda está más cerca, ensanchamos hacia la derecha
                extend_angle = math.atan2(self.car_radius, dist_2)
                extend_idx = int(extend_angle / angle_inc)
                for j in range(max(start_idx, i - extend_idx + 1), i + 1):
                    processed_ranges[j] = min(processed_ranges[j], dist_2)

        # ----------------------------------------------------
        # 3. BURBUJA DE SEGURIDAD ABSOLUTA (Extra protección)
        # ----------------------------------------------------
        min_dist = float('inf')
        closest_idx = -1
        
        for i in range(start_idx, end_idx + 1):
            if 0.0 < processed_ranges[i] < min_dist:
                min_dist = processed_ranges[i]
                closest_idx = i

        # Solo aplicamos la burbuja si el objeto está a menos de 0.3m de riesgo inminente
        if closest_idx != -1 and min_dist > 0.35 and min_dist < 1.0:
            bubble_angle = math.atan2(self.car_radius, min_dist)
            bubble_idx = int(bubble_angle / angle_inc)
            b_start = max(start_idx, closest_idx - bubble_idx)
            b_end = min(end_idx, closest_idx + bubble_idx)
            
            for i in range(b_start, b_end + 1):
                processed_ranges[i] = 0.0

        # ----------------------------------------------------
        # 4. EXTRACCIÓN DEL MAX-GAP
        # ----------------------------------------------------
        max_gap_length = 0
        max_gap_start = 0
        max_gap_end = 0

        current_gap_start = -1
        current_gap_length = 0

        for i in range(start_idx, end_idx + 1):
            if processed_ranges[i] > 0.1:  # Ignoramos muros virtuales de 0.0 y distancias fantasma
                if current_gap_start == -1:
                    current_gap_start = i
                current_gap_length += 1
            else:
                if current_gap_length > max_gap_length:
                    max_gap_length = current_gap_length
                    max_gap_start = current_gap_start
                    max_gap_end = i - 1
                current_gap_start = -1
                current_gap_length = 0

        if current_gap_length > max_gap_length:
            max_gap_start = current_gap_start
            max_gap_end = end_idx
            
        # ----------------------------------------------------
        # 5. SELECCIÓN DEL OBJETIVO ESTABLE
        # ----------------------------------------------------
        # Gracias al Disparity Extender, los bordes de los obstáculos ya están 
        # inflados hacia el espacio libre. Podemos apuntar a la máxima 
        # profundidad sin miedo a rozar las esquinas
        best_idx = max_gap_start
        max_depth = 0.0
        
        for i in range(max_gap_start, max_gap_end + 1):
            if processed_ranges[i] > max_depth:
                max_depth = processed_ranges[i]
                best_idx = i
                
        target_idx = best_idx
        target_angle = angle_min + (target_idx * angle_inc)
        
        # ----------------------------------------------------
        # 6. ESCÁNER FRONTAL PARA EL CONTROL PD
        # ----------------------------------------------------
        frontal_start_idx = int((-self.frontal_view_angle - angle_min) / angle_inc)
        frontal_end_idx = int((self.frontal_view_angle - angle_min) / angle_inc)
        
        frontal_start_idx = max(0, frontal_start_idx)
        frontal_end_idx = min(len(msg.ranges) - 1, frontal_end_idx)
        
        frontal_dist = float('inf')
        for i in range(frontal_start_idx, frontal_end_idx + 1):
            r = msg.ranges[i]
            if not math.isinf(r) and not math.isnan(r) and r > 0.0:
                if r < frontal_dist:
                    frontal_dist = r

        if math.isinf(frontal_dist):
            frontal_dist = self.max_lidar_range

        # ----------------------------------------------------
        # 7. CÁLCULO DEL CONTROL PD Y PUBLICACIÓN
        # ----------------------------------------------------
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        if dt <= 0.0: dt = 0.01

        error = target_angle
        derivative = (error - self.prev_error) / dt

        self.prev_error = error
        self.last_time = current_time

        target_dist = processed_ranges[target_idx] if processed_ranges[target_idx] > 0 else self.failsafe_dist
    
        # A. VELOCIDAD
        if target_dist >= self.braking_distance_vel:
            speed = self.max_speed
        else:
            dist_ratio_vel = target_dist / self.braking_distance_vel
            exp_factor_vel = (math.exp(self.k_vel * dist_ratio_vel) - 1.0) / (math.exp(self.k_vel))
            speed = self.min_speed + ((self.max_speed - self.min_speed) * exp_factor_vel)

        # B. DIRECCIÓN
        if frontal_dist >= self.braking_distance_kp:
            exp_factor_kp = 1.0 
        else:
            dist_ratio_kp = frontal_dist / self.braking_distance_kp
            exp_factor_kp = (math.exp(self.k_kp * dist_ratio_kp) - 1.0) / (math.exp(self.k_kp))

        dynamic_Kp = self.Kp * (1.0 - (self.steering_attenuation * exp_factor_kp))
        dynamic_Kd = self.Kd * (1.0 - (self.steering_attenuation * exp_factor_kp))
        
        raw_steering = (dynamic_Kp * error) + (dynamic_Kd * derivative)
        steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, raw_steering))

        self.publish_drive(speed, steering_angle)

    def publish_drive(self, speed, steering):
        msg = AckermannDriveStamped()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)
        self.drive_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ReactiveFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_drive(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
