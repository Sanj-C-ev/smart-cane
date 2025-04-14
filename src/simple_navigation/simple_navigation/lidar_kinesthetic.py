#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8MultiArray
import numpy as np

class DirectionalObstacleDetector(Node):
    def __init__(self):
        super().__init__('directional_obstacle_detector')
        
        # Configure sectors with individual thresholds (meters)
        # Format: (name, center_angle_rad, width_rad, threshold_m)
        self.sectors = [
            ('left', np.pi/3, np.pi/6, 0.5),       # 60° left, 30° width, 0.5m threshold
            ('left_near', np.pi/6, np.pi/6, 0.75),   # 30° left, 1.0m threshold
            ('front', 0, np.pi/6, 1.2),             # Center, 2.0m threshold
            ('right_near', -np.pi/6, np.pi/6, 0.75), # 30° right, 1.0m threshold
            ('right', -np.pi/3, np.pi/6, 0.5)       # 60° right, 0.5m threshold
        ]
        
        # ROS 2 interface
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.publisher = self.create_publisher(
            Int8MultiArray,
            '/haptic',
            10
        )
        
        self.get_logger().info("Obstacle detector ready with sector thresholds:")
        for name, _, _, thresh in self.sectors:
            self.get_logger().info(f"  {name}: {thresh}m")

    def scan_callback(self, msg):
        """Process LIDAR data and publish obstacle presence"""
        try:
            # Initialize feedback array [left, left_near, front, right_near, right]
            feedback = [0] * 5
            
            # Convert scan to numpy arrays
            angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
            ranges = np.array(msg.ranges)
            
            # Check each sector
            for i, (_, center, width, threshold) in enumerate(self.sectors):
                # Get sector boundaries (handling angle wrapping)
                start_angle = center - width/2
                end_angle = center + width/2
                
                if start_angle < msg.angle_min:
                    start_angle += 2*np.pi
                    end_angle += 2*np.pi
                
                # Create mask for this sector
                if start_angle > end_angle:
                    mask = (angles >= start_angle) | (angles <= end_angle)
                else:
                    mask = (angles >= start_angle) & (angles <= end_angle)
                
                # Check for obstacles within threshold
                sector_ranges = ranges[mask]
                valid_ranges = sector_ranges[
                    (sector_ranges > 0.1) &  # Ignore very close readings
                    (sector_ranges <= threshold) & 
                    ~np.isinf(sector_ranges)
                ]
                
                if valid_ranges.size > 0:
                    feedback[i] = 1
            
            # Publish result
            feedback_msg = Int8MultiArray()
            feedback_msg.data = feedback
            self.publisher.publish(feedback_msg)
            
            self.get_logger().debug(f"Published feedback: {feedback}")
            
        except Exception as e:
            self.get_logger().error(f"Scan processing error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = DirectionalObstacleDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
