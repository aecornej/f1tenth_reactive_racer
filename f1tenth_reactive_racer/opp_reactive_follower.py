#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class OppReactiveFollowerNode(Node):
    def __init__(self):
        # 1. NOMBRE DEL NODO INDEPENDIENTE
        super().__init__('opp_reactive_follower_node')
        
        # 2. TÓPICOS DEL OPONENTE (Según tu ros2 topic list)
        self.scan_sub = self.create_subscription(LaserScan, '/opp_scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/opp_drive', 10)

        # ==========================================
        # ⚙️ PANEL DE CONTROL Y TUNING (MODO OPONENTE)
        # ==========================================
        
        self.view_angle = 1.3           
        self.frontal_view_angle = 0.4    
        self.max_lidar_range = 30.0

        self.car_radius = 0.42           
        self.disparity_threshold = 0.4  
        self.failsafe_dist = 0.42        

        # 3. VELOCIDADES REDUCIDAS AL ~20%
        self.max_speed = 4.0
        self.min_speed = 1.5
        self.braking_distance_vel = 4.0  

        # 4. CONTROL PD (Ligeramente más suave para bajas velocidades)
        self.braking_distance_kp = 2.1
        self.Kp = 1.0  # Reducido para que no zigzaguee al ir tan lento
        self.k_vel = 1.8
        self.k_kp = 0.2
        self.Kd = 0.05
        self.steering_attenuation = 0.48         
        self.max_steering_angle = 0.5	         

        self.prev_error = 0.0
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info('Piloto OPONENTE (Obstáculo Dinámico) iniciado a 20% de velocidad.')

    def scan_callback(self, msg):
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

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
        
        for i in range(len(processed_ranges)):
            if i < start_idx or i > end_idx:
                processed_ranges[i] = 0.0

        disparities = []
        for i in range(start_idx, end_idx):
            if abs(processed_ranges[i] - processed_ranges[i+1]) > self.disparity_threshold:
                disparities.append(i)

        for i in disparities:
            dist_1 = processed_ranges[i]
            dist_2 = processed_ranges[i+1]
            
            if dist_1 < dist_2:
                extend_angle = math.atan2(self.car_radius, dist_1)
                extend_idx = int(extend_angle / angle_inc)
                for j in range(i + 1, min(end_idx + 1, i + 1 + extend_idx)):
                    processed_ranges[j] = min(processed_ranges[j], dist_1)
            else:
                extend_angle = math.atan2(self.car_radius, dist_2)
                extend_idx = int(extend_angle / angle_inc)
                for j in range(max(start_idx, i - extend_idx + 1), i + 1):
                    processed_ranges[j] = min(processed_ranges[j], dist_2)

        min_dist = float('inf')
        closest_idx = -1
        
        for i in range(start_idx, end_idx + 1):
            if 0.0 < processed_ranges[i] < min_dist:
                min_dist = processed_ranges[i]
                closest_idx = i

        if closest_idx != -1 and min_dist > 0.35 and min_dist < 1.0:
            bubble_angle = math.atan2(self.car_radius, min_dist)
            bubble_idx = int(bubble_angle / angle_inc)
            b_start = max(start_idx, closest_idx - bubble_idx)
            b_end = min(end_idx, closest_idx + bubble_idx)
            
            for i in range(b_start, b_end + 1):
                processed_ranges[i] = 0.0

        max_gap_length = 0
        max_gap_start = 0
        max_gap_end = 0

        current_gap_start = -1
        current_gap_length = 0

        for i in range(start_idx, end_idx + 1):
            if processed_ranges[i] > 0.1:  
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
            
        best_idx = max_gap_start
        max_depth = 0.0
        
        for i in range(max_gap_start, max_gap_end + 1):
            if processed_ranges[i] > max_depth:
                max_depth = processed_ranges[i]
                best_idx = i
                
        target_idx = best_idx
        target_angle = angle_min + (target_idx * angle_inc)
        
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

        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        if dt <= 0.0: dt = 0.01

        error = target_angle
        derivative = (error - self.prev_error) / dt

        self.prev_error = error
        self.last_time = current_time

        target_dist = processed_ranges[target_idx] if processed_ranges[target_idx] > 0 else self.failsafe_dist
    
        if target_dist >= self.braking_distance_vel:
            speed = self.max_speed
        else:
            dist_ratio_vel = target_dist / self.braking_distance_vel
            if self.k_vel == 0.0:
                exp_factor_vel = dist_ratio_vel
            else:
                exp_factor_vel = (math.exp(self.k_vel * dist_ratio_vel) - 1.0) / (math.exp(self.k_vel) - 1.0)
            speed = self.min_speed + ((self.max_speed - self.min_speed) * exp_factor_vel)

        if frontal_dist >= self.braking_distance_kp:
            exp_factor_kp = 1.0 
        else:
            dist_ratio_kp = frontal_dist / self.braking_distance_kp
            if self.k_kp == 0.0:
                exp_factor_kp = dist_ratio_kp
            else:
                exp_factor_kp = (math.exp(self.k_kp * dist_ratio_kp) - 1.0) / (math.exp(self.k_kp) - 1.0)

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
    node = OppReactiveFollowerNode()
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
