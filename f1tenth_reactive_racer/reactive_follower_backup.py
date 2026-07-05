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
        
        # --- 1. PARÁMETROS DE PERCEPCIÓN --- Define cómo el robot ve e interpreta el mundo.
        self.view_angle = 1.1           
        # Si se aumenta: Ve más hacia los lados. Puede confundirse con aperturas laterales en rectas.
        # Si se disminuye: Visión de túnel. Excelente en rectas, pero ciego al entrar a curvas cerradas.
        
        self.frontal_view_angle = 0.40
        # Mide la distancia al muro que está justo frente al morro del auto, determinando cuándo liberar el Kp y Kd para dar el volantazo
        # Si se aumenta: El sensor detectará las paredes laterales de las rectas largas como si fueran obstáculos frontales, provocando que el 
        # volante se vuelva sensible y el auto tiemble antes de llegar a la curva.
    	# Si se disminuye: Visión de láser. Podría ignorar un muro inminente si el auto no llega perfectamente alineado a la curva, lo que causaría 
    	# una reacción tardía del volante y un posible choque.
        
        self.max_lidar_range = 30       
        # Distancia que asume como "libre" cuando el LiDAR devuelve un valor infinito o error. Limite de 30 metros
        # Si se aumenta: Considera pasillos largos como muy seguros y acelera a fondo.
        # Si se disminuye: Comportamiento más conservador y cauteloso ante huecos desconocidos.

        # --- 2. PARÁMETROS DE SEGURIDAD --- Evitan colisiones directas inflando los obstáculos.
        self.bubble_size = 0	# Solo valores enteros            
        # Si se aumenta: El robot se aleja más de las paredes (más seguro, pero hace los giros más abiertos).
        # Si se disminuye: El robot pasa rozando el vértice de la curva (trayectoria más rápida, pero muy riesgoso).
        
        self.failsafe_dist = 0.8         
        # Distancia objetivo de emergencia si el algoritmo falla temporalmente en medir el hueco.

        # --- 3. PARÁMETROS DE VELOCIDAD --- Determinan la aceleración y los puntos de frenada.
        self.braking_distance_vel = 4.0
    	# Distancia (metros) mirando al HUECO para empezar a reducir la velocidad.
        
        self.max_speed = 7.0
        # Si se aumenta: Menor tiempo por vuelta en rectas, pero requiere un Control PD muy bien sintonizado para no chocar.
        
        self.min_speed = 1.5             
        # Velocidad mínima de tránsito garantizada en medio de curvas muy cerradas o evasión extrema.
        

        # --- 4. PARÁMETROS DE DIRECCIÓN (CONTROL PD) --- Manejan la agresividad, anticipación y estabilidad del volante.
        self.braking_distance_kp = 3.5
    	# NUEVO: Distancia (metros) mirando al MURO FRONTAL para liberar el Kp y Kd. 
    	# Mantenerlo menor que braking_distance_vel evita que gire demasiado pronto.
        
        
        self.Kp = 2.22           # Fuerza bruta con la que el volante gira hacia el objetivo. Si es excesivo: El auto zigzaguea violentamente.
        self.k_vel = 2.8	# Constante exponencial para la velocidad (mayor agresividad en en cambio de velocidad en curvas)
        self.k_kp = 8.0		# Constante exponencial para Kp (mayor agresividad en en cambio de dirección en curvas)
        self.Kd = 1.0           # "Amortiguador". Reacciona a los cambios bruscos de la pista. Aumenta para frenar el zigzagueo del Kp
        self.steering_attenuation = 0.25  # (porcentaje). Reduce el valor del Kp únicamente cuando el auto va en rectas.
        
        self.max_steering_angle = 0.9   
        # Unidad: Radianes (con un limite de 1.066 radianes (~61 grados)).
        # Límite físico inamovible. NO superar este valor o las matemáticas del simulador perderán fidelidad.

        # --- VARIABLES DE MEMORIA DEL CONTROLADOR ---
        # Necesarias para calcular la derivada con respecto al tiempo
        self.prev_error = 0.0
        # self.get_clock().now().nanoseconds / 1e9 nos entrega el tiempo en segundos
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info('Piloto Follow the Gap PD iniciado. Parámetros cargados correctamente.')

    def scan_callback(self, msg):
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        # ----------------------------------------------------
        # 1. PERCEPCIÓN: Recorte del campo de visión y limpieza
        # ----------------------------------------------------
        processed_ranges = []
        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r):
                processed_ranges.append(self.max_lidar_range)
            else:
                processed_ranges.append(r)

        start_idx = int((-self.view_angle - angle_min) / angle_inc)
        end_idx = int((self.view_angle - angle_min) / angle_inc)

        start_idx = max(0, start_idx)
        end_idx = min(len(processed_ranges) - 1, end_idx)

        for i in range(len(processed_ranges)):
            if i < start_idx or i > end_idx:
                processed_ranges[i] = 0.05

        # ----------------------------------------------------
        # 2. BURBUJA DE SEGURIDAD
        # ----------------------------------------------------
        min_dist = float('inf')
        closest_idx = start_idx
        
        for i in range(start_idx, end_idx + 1):
            if processed_ranges[i] > 0.0 and processed_ranges[i] < min_dist:
                min_dist = processed_ranges[i]
                closest_idx = i

        b_start = max(0, closest_idx - self.bubble_size)
        b_end = min(len(processed_ranges) - 1, closest_idx + self.bubble_size)
        for i in range(b_start, b_end + 1):
            processed_ranges[i] = 0.0

        # ----------------------------------------------------
        # 3. PLANIFICACIÓN: Encontrar el Gap
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

        if current_gap_length > max_gap_length:
            max_gap_start = current_gap_start
            max_gap_end = end_idx

        # ----------------------------------------------------
    	# ESCÁNER DE PARED FRONTAL (Para despertar el Kp/Kd)
    	# ----------------------------------------------------
    	# Calculamos los índices exclusivamente para nuestro cono frontal estrecho
        frontal_start_idx = int((-self.frontal_view_angle - angle_min) / angle_inc)
        frontal_end_idx = int((self.frontal_view_angle - angle_min) / angle_inc)
        
        frontal_start_idx = max(0, frontal_start_idx)
        frontal_end_idx = min(len(msg.ranges) - 1, frontal_end_idx)
        
        frontal_dist = float('inf')
    	# Leemos los datos CRUDOS del LiDAR (msg.ranges), ignorando la burbuja artificial
        for i in range(frontal_start_idx, frontal_end_idx + 1):
        	r = msg.ranges[i]
        	if not math.isinf(r) and not math.isnan(r):
         	   if r > 0.0 and r < frontal_dist:
        	        frontal_dist = r

        if math.isinf(frontal_dist):
       		frontal_dist = self.max_lidar_range
        
        
        # ----------------------------------------------------
        # 4. CONTROL PD: Apuntar al Gap y ajustar velocidad
        # ----------------------------------------------------
        target_idx = int((max_gap_start + max_gap_end) / 2)
        target_angle = angle_min + (target_idx * angle_inc)

        # Cálculo de la Derivada (dError / dt) usando el reloj de ROS 2
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        
        # Prevenir división por cero, especialmente en el primer callback
        if dt <= 0.0:
            dt = 0.01

        # El error en nuestro sistema reactivo es el ángulo hacia donde queremos apuntar
        error = target_angle
        
        # Qué tan violento fue el cambio de dirección del hueco respecto al ciclo anterior
        derivative = (error - self.prev_error) / dt

        # Guardar valores para el siguiente ciclo
        self.prev_error = error
        self.last_time = current_time

        # --- A. DINÁMICA DE VELOCIDAD (Mira al Gap) ---
        target_dist = processed_ranges[target_idx] if processed_ranges[target_idx] > 0 else self.failsafe_dist
    
        if target_dist >= self.braking_distance_vel:
        	speed = self.max_speed
        else:
        	dist_ratio_vel = target_dist / self.braking_distance_vel
        	exp_factor_vel = (math.exp(self.k_vel * dist_ratio_vel) - 1.0) / (math.exp(self.k_vel) - 1.0)
        	speed = self.min_speed + ((self.max_speed - self.min_speed) * exp_factor_vel)

    	# --- B. DINÁMICA DE DIRECCIÓN (Mira a la pared frontal) ---
        if frontal_dist >= self.braking_distance_kp:
        	# Si no hay pared enfrente, aplicamos la atenuación máxima para ir rectos y estables
        	exp_factor_kp = 1.0 
        else:
        	# Si la pared se acerca por el frente, el factor se desploma hacia 0
        	dist_ratio_kp = frontal_dist / self.braking_distance_kp
        	exp_factor_kp = (math.exp(self.k_kp * dist_ratio_kp) - 1.0) / (math.exp(self.k_kp) - 1.0)


        # --- OPTIMIZACIÓN DE DIRECCIÓN CON SEÑAL PD ---
        
        dynamic_Kp = self.Kp * (1.0 - (self.steering_attenuation * exp_factor_kp))
        dynamic_Kd = self.Kd * (1.0 - (self.steering_attenuation * exp_factor_kp))
        
        # Ecuación central del Control Proporcional-Derivativo
        raw_steering = (dynamic_Kp * error) + (dynamic_Kd * derivative)

        # Saturación Física (Clipping)
        steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, raw_steering))

        self.publish_drive(speed, steering_angle)
        #self.publish_drive(speed, raw_steering)

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
