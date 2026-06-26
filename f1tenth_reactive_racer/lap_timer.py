#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import math

class LapTimerNode(Node):
    def __init__(self):
        super().__init__('lap_timer_node')
        
        # Suscripción con el QoS correcto para escuchar sensores de simulación
        self.subscription = self.create_subscription(
            Odometry, 
            '/ego_racecar/odom', 
            self.odom_callback, 
            qos_profile_sensor_data
        )

        # Coordenadas exactas donde teletransportamos el auto en la pista Budapest
        self.start_x = -12.311
        self.start_y = 9.941

        # Variables del cronómetro y contador
        self.lap_count = 0
        self.max_laps = 10
        
        # Inicializamos en 0.0. Se llenarán cuando el auto empiece a moverse
        self.lap_start_time = 0.0
        self.total_start_time = 0.0

        # Estados de seguridad
        self.race_started = False         # Nueva bandera para saber si ya arrancó el motor
        self.has_left_start_line = False
        self.min_distance_to_leave = 5.0  # El auto debe alejarse 5 metros para armar el sensor de meta
        self.finish_line_radius = 2.0     # Radio en metros para considerar que cruzó la meta

        self.get_logger().info('⏱️ Juez de carrera iniciado. Esperando a que el auto comience a moverse...')

    def odom_callback(self, msg):
        if self.lap_count >= self.max_laps:
            return

        # Extraemos la posición X e Y actual del auto
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # Extraemos el tiempo exacto del motor de físicas del simulador
        current_time = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)

        # Teorema de Pitágoras para saber a qué distancia estamos del punto de partida
        distance_to_start = math.sqrt((current_x - self.start_x)**2 + (current_y - self.start_y)**2)

        # 0. Detectar el inicio real de la carrera
        if not self.race_started:
            # Si el auto se ha movido más de 10 cm desde su punto inicial, arranca el cronómetro
            if distance_to_start > 0.1:
                self.race_started = True
                self.lap_start_time = current_time
                self.total_start_time = current_time
                self.get_logger().info('🏁 ¡Movimiento detectado! Cronómetro de la vuelta 1 corriendo...')
            return # Salimos del callback para no evaluar la meta en este mismo instante

        # 1. Detectar si el auto ya salió de la zona de meta (5 metros)
        if not self.has_left_start_line and distance_to_start > self.min_distance_to_leave:
            self.has_left_start_line = True

        # 2. Detectar si el auto volvió y cruzó la meta
        if self.has_left_start_line and distance_to_start < self.finish_line_radius:
            lap_time = current_time - self.lap_start_time
            self.lap_count += 1
            
            self.get_logger().info(f'🏎️ ¡Vuelta {self.lap_count} completada! Tiempo: {lap_time:.2f} segundos')

            self.lap_start_time = current_time
            self.has_left_start_line = False

            if self.lap_count == self.max_laps:
                total_time = current_time - self.total_start_time
                self.get_logger().info(f'🏆 ¡CARRERA TERMINADA! {self.max_laps} vueltas en {total_time:.2f} segundos.')

def main(args=None):
    rclpy.init(args=args)
    node = LapTimerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
