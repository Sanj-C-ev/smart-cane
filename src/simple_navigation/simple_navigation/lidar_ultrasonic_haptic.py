#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Int8MultiArray
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, Circle
import math

class MultiSensorObstacleVisualizer(Node):
    def __init__(self):
        super().__init__('multi_sensor_obstacle_visualizer')
        
        # Sensor configuration
        self.ultrasonic_cluster_offset = 0.75  # 75cm ahead
        self.single_ultrasonic_offset = 0.30   # 30cm ahead
        self.us_1 = 0.35
        self.us_2 = 0.75
        
        # Sector configuration
        self.sectors = [
            ('left', np.pi/3, np.pi/6, 0.5, 'red'),     # 60° left
            ('left_near', np.pi/6, np.pi/6, 0.75, 'orange'),  # 30° left
            ('front', 0, np.pi/6, 1.2, 'green'),        # Center
            ('right_near', -np.pi/6, np.pi/6, 0.75, 'orange'), # 30° right
            ('right', -np.pi/3, np.pi/6, 0.5, 'red')    # 60° right
        ]
        
        # ROS interfaces
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        
        # Two separate ultrasonic topics as specified
        self.ultrasonic_cluster_sub = self.create_subscription(
            Float32MultiArray, '/sensors/ultrasonic_cluster_1', 
            self.ultrasonic_cluster_callback, 10)
        self.single_ultrasonic_sub = self.create_subscription(
            Float32MultiArray, '/sensors/ultrasonic_cluster_2', 
            self.single_ultrasonic_callback, 10)
        
        self.haptic_pub = self.create_publisher(
            Int8MultiArray, '/haptic', 10)
        
        # Sensor data storage
        self.lidar_data = None
        self.ultrasonic_cluster_data = [float('inf')] * 3  # [left, front, right]
        self.single_ultrasonic_data = float('inf')
        self.detection_result = [0] * 5
        
        # Visualization setup
        plt.ion()
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.setup_visualization()
        
        # Control timer
        self.timer = self.create_timer(0.1, self.update_detection)

    def setup_visualization(self):
        """Initialize the visualization window"""
        self.ax.clear()
        self.ax.set_theta_offset(np.pi/2)  # 0° at top
        self.ax.set_ylim(0, 2.0)  # Show up to 2 meters
        self.ax.set_title("Multi-Sensor Obstacle Detection\n(Red=Obstacle)")
        
        # Create sector visualizations
        self.sector_artists = []
        for name, center, width, threshold, color in self.sectors:
            wedge = Wedge(
                (0,0), threshold, 
                np.degrees(center-width/2), 
                np.degrees(center+width/2),
                color=color, alpha=0.2
            )
            self.sector_artists.append(self.ax.add_patch(wedge))
            self.ax.text(center, threshold*0.8, name, ha='center')
        
        # Ultrasonic positions
        self.us_left_artist, = self.ax.plot([], [], 'ro', markersize=8)
        self.us_front_artist, = self.ax.plot([], [], 'go', markersize=8)
        self.us_right_artist, = self.ax.plot([], [], 'bo', markersize=8)
        self.us_single_artist, = self.ax.plot([], [], 'mo', markersize=10)
        
        # Detection indicators
        self.detection_artists = []
        for i, (_, center, _, _, _) in enumerate(self.sectors):
            circle = Circle((center, 0.15), 0.05, color='red', alpha=0)
            self.detection_artists.append(self.ax.add_patch(circle))
        
        # Legend
        self.fig.tight_layout()
        self.fig.canvas.draw()

    def lidar_callback(self, msg):
        """Store LIDAR data"""
        self.lidar_data = msg

    def ultrasonic_cluster_callback(self, msg):
        """Store 3-sensor ultrasonic cluster data [left, front, right]"""
        if len(msg.data) == 3:
            self.ultrasonic_cluster_data = msg.data

    def single_ultrasonic_callback(self, msg):
        """Store single ultrasonic sensor data"""
        if len(msg.data) == 1:
            self.single_ultrasonic_data = msg.data[0]

    def update_detection(self):
        """Process sensor data and update visualization"""
        try:
            feedback = [0] * 5
            
            # Get ultrasonic data from both topics
            us_left, us_front, us_right = self.ultrasonic_cluster_data
            us_single = self.single_ultrasonic_data
            
            # Update ultrasonic positions in visualization
            self.us_left_artist.set_data([np.pi/3], [min(us_left, 2.0)])
            self.us_front_artist.set_data([0], [min(us_front, 2.0)])
            self.us_right_artist.set_data([-np.pi/3], [min(us_right, 2.0)])
            self.us_single_artist.set_data([0], [min(us_single, 2.0)])
            us_1_threshold = self.us_1
            us_2_threshold = self.us_2
            # Check ultrasonic detections (cluster first)
            if us_left <= us_1_threshold:
                feedback[1] = 1  # Left sector
            if us_front <= us_1_threshold:
                feedback[2] = 1  # Front sector
            if us_right <= us_1_threshold:
                feedback[3] = 1  # Right sector
            
            # Check single ultrasonic (highest priority)
            if us_single <= us_2_threshold:
                feedback[2] = 1  # Front sector
            
            # Process LIDAR if no close obstacles detected by ultrasonics
            if sum(feedback) == 0 and self.lidar_data is not None:
                angles = np.linspace(
                    self.lidar_data.angle_min,
                    self.lidar_data.angle_max,
                    len(self.lidar_data.ranges))
                
                ranges = np.array(self.lidar_data.ranges)
                
                for i, (_, center, width, threshold, _) in enumerate(self.sectors):
                    start_angle = center - width/2
                    end_angle = center + width/2
                    
                    if start_angle < self.lidar_data.angle_min:
                        start_angle += 2*np.pi
                        end_angle += 2*np.pi
                    
                    if start_angle > end_angle:
                        mask = (angles >= start_angle) | (angles <= end_angle)
                    else:
                        mask = (angles >= start_angle) & (angles <= end_angle)
                    
                    sector_ranges = ranges[mask]
                    valid_ranges = sector_ranges[
                        (sector_ranges > 0.1) & 
                        (sector_ranges <= threshold) & 
                        ~np.isinf(sector_ranges)
                    ]
                    
                    if valid_ranges.size > 0:
                        feedback[i] = 1
            
            # Update detection visualization
            for i, artist in enumerate(self.detection_artists):
                artist.set_alpha(feedback[i] * 0.8)  # 0 if no detection, 0.8 if detected
            
            # Publish haptic feedback
            feedback_msg = Int8MultiArray()
            feedback_msg.data = feedback
            self.haptic_pub.publish(feedback_msg)
            self.detection_result = feedback
            
            # Update plot
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            
        except Exception as e:
            self.get_logger().error(f"Detection error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = MultiSensorObstacleVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        plt.close('all')
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()