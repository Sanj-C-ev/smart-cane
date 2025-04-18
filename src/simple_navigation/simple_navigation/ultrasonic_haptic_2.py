#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class SimpleObstacleDetector(Node):
    def __init__(self):
        super().__init__('simple_obstacle_detector')
        ultrasonic_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        # Threshold distances (in meters)
        self.min_threshold = 0.05  # Minimum valid distance
        self.front_threshold = 0.4  # Front sensor threshold
        self.rear_threshold = 0.8   # Rear sensor threshold
        
        # ROS interfaces
        self.ultrasonic_cluster_sub = self.create_subscription(
            Float32MultiArray, '/sensors/ultrasonic_cluster_1', 
            self.ultrasonic_cluster_callback, ultrasonic_qos)
        self.rear_ultrasonic_sub = self.create_subscription(
            Float32MultiArray, '/sensors/ultrasonic_cluster_2', 
            self.rear_ultrasonic_callback, ultrasonic_qos)
        
        self.haptic_pub = self.create_publisher(Int8MultiArray, '/haptic', 10)
        
        # Sensor data storage
        self.front_sensor_data = float('inf')
        self.left_sensor_data = float('inf')
        self.right_sensor_data = float('inf')
        self.rear_sensor_data = float('inf')

    def ultrasonic_cluster_callback(self, msg):
        """Store front sensor data from cluster 1 (assuming index 1 is front)"""
        if len(msg.data) >= 2:  # Ensure we have at least 2 sensors (left, front)

            self.front_sensor_data = msg.data[1]  # Second element is front sensor
            self.detect_obstacles()

    def rear_ultrasonic_callback(self, msg):
        """Store single sensor data from cluster 2"""
        if len(msg.data) >= 1:
            self.rear_sensor_data = msg.data[0]
            self.detect_obstacles()

    def detect_obstacles(self):
        """Detect obstacles and publish feedback"""
        feedback = [0] * 5  # Initialize [a,b,c,d,e]
        
        # Front sensor detection (cluster 1)
        if self.min_threshold <= self.front_sensor_data <= self.front_threshold:
            feedback = [0, 0, 0, 1, 1]  # Pattern for front obstacle
        
        # Rear sensor detection (cluster 2)
        elif self.min_threshold <= self.rear_sensor_data <= self.rear_threshold:
            feedback = [1, 1, 0, 0, 0]  # Pattern for rear obstacle
        
        # Publish haptic feedback
        feedback_msg = Int8MultiArray()
        feedback_msg.data = feedback
        self.haptic_pub.publish(feedback_msg)
        
        self.get_logger().info(f"Published feedback: {feedback}")
    

def main(args=None):
    rclpy.init(args=args)
    node = SimpleObstacleDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()