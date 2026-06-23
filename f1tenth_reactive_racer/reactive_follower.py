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
        # Rol: Define cómo el robot ve e interpreta el mundo.
        self.view_angle = 1.25           
        # Unidad: Radianes (~70 grados a cada lado). Rango sugerido: 0.8 a 1.5.
        # Si se aumenta: Ve más hacia los lados. Puede confundirse con aperturas laterales en rectas.
        # Si se disminuye: Visión de túnel. Excelente en rectas, pero ciego al entrar a curvas cerradas.
        
        self.max_lidar_range = 8.0       
        # Unidad: Metros. Rango sugerido: 3.0 a 10.0.
        # Rol: Distancia que asume como "libre" cuando el LiDAR devuelve un valor infinito o error.
        # Si se aumenta: Considera pasillos largos como muy seguros y acelera a fondo.
        # Si se disminuye: Comportamiento más conservador y cauteloso ante huecos desconocidos.

        # --- 2. PARÁMETROS DE SEGURIDAD ---
        # Rol: Evitan colisiones directas inflando los obstáculos.
        self.bubble_size = 18            
        # Unidad: Número de rayos del sensor. Rango sugerido: 10 a 30.
        # Si se aumenta: El robot se aleja más de las paredes (más seguro, pero hace los giros más abiertos).
        # Si se disminuye: El robot pasa rozando el vértice de la curva (trayectoria más rápida, pero muy riesgoso).
        
        self.failsafe_dist = 0.8         
        # Unidad: Metros. Rango sugerido: 0.5 a 1.5.
        # Rol: Distancia objetivo de emergencia si el algoritmo falla temporalmente en medir el hueco.

        # --- 3. PARÁMETROS DE VELOCIDAD ---
        # Rol: Determinan la aceleración y los puntos de frenada.
        self.max_speed = 3.5             
        # Unidad: Metros por segundo (m/s). Rango sugerido: 3.0 a 6.0.
        # Si se aumenta: Menor tiempo por vuelta en rectas, pero requiere un Control PD muy bien sintonizado para no chocar.
        
        self.min_speed = 1.5             
        # Unidad: m/s. Rango sugerido: 0.5 a 2.0.
        # Rol: Velocidad mínima de tránsito garantizada en medio de curvas muy cerradas o evasión extrema.
        
        self.braking_distance = 4.5      
        # Unidad: Metros. Rango sugerido: 2.0 a 7.0.
        # Si se aumenta: Frena con mucha anticipación. Es más suave y estable, pero pierdes tiempo valioso.
        # Si se disminuye: Frenada de competencia (apurada). Si es muy bajo, la inercia sacará al auto de la pista.

        # --- 4. PARÁMETROS DE DIRECCIÓN (CONTROL PD) ---
        # Rol: Manejan la agresividad, anticipación y estabilidad del volante.
        self.Kp = 1.8                    
        # Ganancia Proporcional. Rango sugerido: 1.0 a 3.0.
        # Rol: La fuerza bruta con la que el volante gira hacia el objetivo.
        # Si se aumenta: Reacción mucho más rápida e inmediata.
        # Si es excesivo: El auto entra en resonancia y zigzaguea violentamente.
        
        # Constante k de agresividad de la curva (puedes jugar con valores entre 2.0 y 5.0)
        self.k_exp = 5.0
        
        self.Kd = 0.5                    
        # Ganancia Derivativa. Rango sugerido: 0.1 a 1.5.
        # Rol: "Amortiguador" predictivo. Reacciona a los cambios bruscos de la pista.
        # Si se aumenta: Frena el zigzagueo del Kp y ayuda a meter la "nariz" en la curva una fracción de segundo antes.
        # Si es excesivo: El auto se vuelve rígido, tembloroso y resiste los giros.
        
        self.steering_attenuation = 0.45  
        # Factor de atenuación. Rango sugerido: 0.2 a 0.8 (Representa un porcentaje).
        # Rol: Reduce el valor del Kp únicamente cuando el auto va a máxima velocidad en rectas.
        # Si se aumenta: El volante se vuelve menos sensible a alta velocidad, garantizando trayectoria recta.
        
        self.max_steering_angle = 0.55   
        # Unidad: Radianes (~31 grados).
        # Rol: Límite físico inamovible. NO superar este valor o las matemáticas del simulador perderán fidelidad.

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

        # Perfil Cinemático de Velocidad (Formula exponencial normalizada)
        target_dist = processed_ranges[target_idx] if processed_ranges[target_idx] > 0 else self.failsafe_dist
        
        if target_dist >= self.braking_distance:
            speed = self.max_speed
        else:
            dist_ratio = target_dist / self.braking_distance
            numerador = math.exp(self.k_exp * dist_ratio) - 1.0
            denominador = math.exp(self.k_exp) - 1.0
            exp_factor = numerador / denominador
            speed = self.min_speed + ((self.max_speed - self.min_speed) * exp_factor)

        # --- OPTIMIZACIÓN DE DIRECCIÓN CON SEÑAL PD ---
        
        speed_factor = speed / self.max_speed 
        dynamic_Kp = self.Kp * (1.0 - (self.steering_attenuation * speed_factor)) 
        
        # Ecuación central del Control Proporcional-Derivativo
        raw_steering = (dynamic_Kp * error) + (self.Kd * derivative)

        # Saturación Física (Clipping)
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
