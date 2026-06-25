#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import time

class LapTimerNode(Node):
    def __init__(self):
        super().__init__('lap_timer_node')
        # Nos suscribimos a la odometría para saber la posición exacta
        self.subscription = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        # Coordenadas exactas donde teletransportamos el auto en la pista Budapest
        self.start_x = -12.311
        self.start_y = 9.941

        # Variables del cronómetro y contador
        self.lap_count = 0
        self.max_laps = 10
        self.lap_start_time = time.time()
        self.total_start_time = time.time()

        # Estados de seguridad para no contar la misma vuelta dos veces seguidas
        self.has_left_start_line = False
        self.min_distance_to_leave = 5.0  # El auto debe alejarse 5 metros para validar la salida
        self.finish_line_radius = 2.0     # Radio en metros para considerar que cruzó la meta

        self.get_logger().info('⏱️ Juez de carrera iniciado. Esperando a que el auto arranque...')

    def odom_callback(self, msg):
        # Si ya terminamos la carrera, ignoramos las demás lecturas
        if self.lap_count >= self.max_laps:
            return

        # Extraemos la posición X e Y actual del auto
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # Teorema de Pitágoras para saber a qué distancia estamos de la meta
        distance_to_start = math.sqrt((current_x - self.start_x)**2 + (current_y - self.start_y)**2)

        # 1. Detectar si el auto ya salió de la zona de meta
        if not self.has_left_start_line and distance_to_start > self.min_distance_to_leave:
            self.has_left_start_line = True
            if self.lap_count == 0:
                self.get_logger().info('🏁 ¡Carrera iniciada! Cronómetro de la vuelta 1 corriendo...')

        # 2. Detectar si el auto volvió y cruzó la meta
        if self.has_left_start_line and distance_to_start < self.finish_line_radius:
            current_time = time.time()
            lap_time = current_time - self.lap_start_time
            self.lap_count += 1
            
            self.get_logger().info(f'🏎️ ¡Vuelta {self.lap_count} completada! Tiempo: {lap_time:.2f} segundos')

            # Reiniciar cronómetro y seguros para la siguiente vuelta
            self.lap_start_time = current_time
            self.has_left_start_line = False

            # Verificar si ya completó todas las vueltas
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
