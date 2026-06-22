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
        
        # --- 1. PARÁMETROS DE PERCEPCIÓN ---
        self.view_angle = 1.22           # Ángulo de visión frontal en radianes (~70 grados).
        self.max_lidar_range = 10.0      # Distancia (metros) que se asume libre si el LiDAR devuelve 'inf'.

        # --- 2. PARÁMETROS DE SEGURIDAD ---
        self.bubble_size = 20            # Cantidad de rayos a hacer "0" alrededor del obstáculo (ancho del chasis).
        self.failsafe_dist = 0.5         # Distancia mínima de seguridad (metros) si el sensor falla en el hueco.

        # --- 3. PARÁMETROS DE VELOCIDAD ---
        self.max_speed = 4.0             # Velocidad máxima en rectas largas (m/s).
        self.min_speed = 1.5             # Velocidad mínima al tomar curvas o sortear obstáculos (m/s).
        self.braking_distance = 3.0      # Distancia (metros) a la que el robot empieza a reducir la velocidad.

        # --- 4. PARÁMETROS DE DIRECCIÓN ---
        self.Kp = 1.5                    # Ganancia Proporcional del volante. Mayor = reacción más agresiva.

        self.get_logger().info('Piloto Follow the Gap iniciado. Parámetros cargados correctamente.')

    def scan_callback(self, msg):
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        # ----------------------------------------------------
        # 1. PERCEPCIÓN: Recorte del campo de visión y limpieza
        # ----------------------------------------------------
        processed_ranges = []
        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r):
                processed_ranges.append(self.max_lidar_range) # Usamos el parámetro de rango máximo
            else:
                processed_ranges.append(r)

        # Calcular los índices de la matriz que corresponden a nuestra visión frontal
        start_idx = int((-self.view_angle - angle_min) / angle_inc)
        end_idx = int((self.view_angle - angle_min) / angle_inc)

        # Validar límites para evitar errores de índice
        start_idx = max(0, start_idx)
        end_idx = min(len(processed_ranges) - 1, end_idx)

        # "Cegamos" temporalmente los láseres laterales y traseros poniéndolos a 0
        for i in range(len(processed_ranges)):
            if i < start_idx or i > end_idx:
                processed_ranges[i] = 0.0

        # ----------------------------------------------------
        # 2. BURBUJA DE SEGURIDAD (Inflar el obstáculo inminente)
        # ----------------------------------------------------
        min_dist = float('inf')
        closest_idx = start_idx
        
        # Encontrar el punto más cercano en nuestro cono frontal
        for i in range(start_idx, end_idx + 1):
            if processed_ranges[i] > 0.0 and processed_ranges[i] < min_dist:
                min_dist = processed_ranges[i]
                closest_idx = i

        # Crear la burbuja haciendo 0 a los rayos vecinos usando el parámetro bubble_size
        b_start = max(0, closest_idx - self.bubble_size)
        b_end = min(len(processed_ranges) - 1, closest_idx + self.bubble_size)
        for i in range(b_start, b_end + 1):
            processed_ranges[i] = 0.0

        # ----------------------------------------------------
        # 3. PLANIFICACIÓN: Encontrar el Gap (hueco) más grande
        # ----------------------------------------------------
        max_gap_length = 0
        max_gap_start = 0
        max_gap_end = 0

        current_gap_start = -1
        current_gap_length = 0

        for i in range(start_idx, end_idx + 1):
            if processed_ranges[i] > 0.0:
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

        # Revisión final por si el hueco termina al borde del arreglo
        if current_gap_length > max_gap_length:
            max_gap_start = current_gap_start
            max_gap_end = end_idx

        # ----------------------------------------------------
        # 4. CONTROL: Apuntar al Gap y ajustar velocidad
        # ----------------------------------------------------
        # Apuntamos al centro matemático del hueco libre
        target_idx = int((max_gap_start + max_gap_end) / 2)
        target_angle = angle_min + (target_idx * angle_inc)

        # Control PD (Proporcional simple) usando Kp
        steering_angle = self.Kp * target_angle

        # Velocidad Dinámica usando los parámetros personalizables
        target_dist = processed_ranges[target_idx] if processed_ranges[target_idx] > 0 else self.failsafe_dist
        
        if target_dist >= self.braking_distance:
            speed = self.max_speed
        else:
            # Perfil Cinemático (Raíz Cuadrada) para frenado de carreras
            dist_ratio = target_dist / self.braking_distance
            speed = self.min_speed + ((self.max_speed - self.min_speed) * math.sqrt(dist_ratio))

        # --- OPTIMIZACIÓN DE DIRECCIÓN ---
        
        # 1. Atenuación por velocidad: El Kp base se reduce si vamos muy rápido
        # Usamos (speed / self.max_speed) para saber qué porcentaje del acelerador estamos usando
        speed_factor = speed / self.max_speed 
        
        # Ajustamos el Kp dinámicamente. (Si vamos a tope, Kp baja a la mitad. Si vamos lento, Kp se mantiene alto)
        dynamic_Kp = self.Kp * (1.0 - (0.5 * speed_factor)) 
        
        raw_steering = dynamic_Kp * target_angle

        # 2. Saturación Física (Clipping)
        # El F1TENTH físicamente no puede girar las llantas más de ~0.54 radianes (aprox 31 grados).
        # Limitar matemáticamente la señal evita comportamientos inestables en el simulador.
        max_steering_physical = 0.54 
        steering_angle = max(-max_steering_physical, min(max_steering_physical, raw_steering))

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
