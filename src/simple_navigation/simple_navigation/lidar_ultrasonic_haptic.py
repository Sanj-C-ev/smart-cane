#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8MultiArray
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class SmartCaneNavigator(Node):
    def __init__(self):
        super().__init__('smart_cane_navigator')
        
        # Angular sectors with different thresholds (angles in radians)
        self.sectors = [
            # (name, center_angle, width, threshold)
            ('hard_left', math.pi/2, math.pi/3, 0.55),    # 60-90° left, 45cm
            ('left', math.pi/4, math.pi/6, 0.55),         # 30-60° left, 45cm
            ('front', 0, math.pi/3, 1.25),                 # 30° left to 30° right, 1m
            ('right', -math.pi/4, math.pi/6, 0.55),       # 30-60° right, 45cm
            ('hard_right', -math.pi/2, math.pi/3, 0.55)   # 60-90° right, 45cm
        ]
        
        # Ultrasonic configuration
        self.us_angles = [math.pi/4, 0, -math.pi/4]  # Left, Front, Right
        self.us_thresholds = [0.3, 0.35, 0.3]  # Left, Front, Right thresholds
        
        # ROS interfaces
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        
        us_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.us_sub = self.create_subscription(
            Float32MultiArray, '/sensors/ultrasonic_cluster_1',
            self.us_callback, us_qos)
            
        self.haptic_pub = self.create_publisher(
            Int8MultiArray, '/haptic', 10)
        
        # Data storage
        self.lidar_ranges = None
        self.lidar_angles = None
        self.us_data = [float('inf')] * 3
        
        # Control timer
        self.timer = self.create_timer(0.1, self.calculate_feedback)

    def lidar_callback(self, msg):
        """Store LIDAR data with proper angle handling"""
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        # Focus on front 180° and invert angle direction if needed
        front_mask = (angles >= -math.pi/2) & (angles <= math.pi/2)
        self.lidar_angles = angles[front_mask]
        self.lidar_ranges = np.array(msg.ranges)[front_mask]

    def us_callback(self, msg):
        """Store ultrasonic data"""
        if len(msg.data) == 3:
            self.us_data = msg.data

    def check_sector_obstacle(self, center_angle, width, threshold):
        """Return 1 if obstacle found in sector"""
        if self.lidar_ranges is None:
            return 0
            
        start_angle = center_angle - width/2
        end_angle = center_angle + width/2
        
        # Handle angle wrapping if needed
        if start_angle < -math.pi:
            mask = (self.lidar_angles >= (start_angle + 2*math.pi)) | (self.lidar_angles <= end_angle)
        elif end_angle > math.pi:
            mask = (self.lidar_angles >= start_angle) | (self.lidar_angles <= (end_angle - 2*math.pi))
        else:
            mask = (self.lidar_angles >= start_angle) & (self.lidar_angles <= end_angle)
        
        sector_ranges = self.lidar_ranges[mask]
        valid_ranges = sector_ranges[
            ~np.isinf(sector_ranges) & 
            (sector_ranges > 0.1) & 
            (sector_ranges < threshold)
        ]
        return 1 if len(valid_ranges) > 0 else 0

    def calculate_feedback(self):
        """Calculate haptic feedback with proper directionality"""
        try:
            haptic = [0] * 5  # [hard_left, left, front, right, hard_right]
            
            # Check LIDAR sectors
            haptic[0] = self.check_sector_obstacle(*self.sectors[0][1:])  # hard_left
            haptic[1] = self.check_sector_obstacle(*self.sectors[1][1:])  # left
            haptic[2] = self.check_sector_obstacle(*self.sectors[2][1:])  # front
            haptic[3] = self.check_sector_obstacle(*self.sectors[3][1:])  # right
            haptic[4] = self.check_sector_obstacle(*self.sectors[4][1:])  # hard_right
            
            # Apply ultrasonic overrides
            if self.us_data[0] < self.us_thresholds[0]:  # Left US
                haptic[0] = 1  # hard_left
                haptic[1] = 1  # left
            if self.us_data[1] < self.us_thresholds[1]:  # Front US
                haptic[2] = 1  # front
            if self.us_data[2] < self.us_thresholds[2]:  # Right US
                haptic[3] = 1  # right
                haptic[4] = 1  # hard_right
            
            # Publish haptic feedback (1 = obstacle present in this direction)
            msg = Int8MultiArray()
            msg.data = haptic
            self.haptic_pub.publish(msg)
            
            self.get_logger().info(
                f"Haptic: {haptic} | "
                f"LIDAR: {[self.check_sector_obstacle(*s[1:]) for s in self.sectors]} | "
                f"US: L={self.us_data[0]:.2f}m, F={self.us_data[1]:.2f}m, R={self.us_data[2]:.2f}m",
                throttle_duration_sec=1.0
            )
            
        except Exception as e:
            self.get_logger().error(f"Navigation error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = SmartCaneNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()