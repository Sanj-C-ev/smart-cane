#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8MultiArray
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, Circle
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class MultiSensorObstacleVisualizer(Node):
    def __init__(self):
        super().__init__('multi_sensor_obstacle_visualizer')
        ultrasonic_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        # Sensor configuration
        self.front_cluster_offset = 0.5  # 50cm ahead of the rear sensor
        self.rear_sensor_offset = -0.5   # 50cm behind the front cluster
        self.min_threshold = 0.2  # Minimum valid distance
        
        # Ultrasonic sensor angles (45° left, 0° front, 45° right)
        self.us_angles = [math.pi/4, 0, -math.pi/4]
        self.rear_us_angle = 0  # Also facing forward
        
        # LIDAR sector configuration (5 sectors covering 180° front)
        self.lidar_sectors = [
            ('left', math.pi/2, math.pi/4, 0.5, 'red'),       # 90°-45° left
            ('near_left', math.pi/4, math.pi/6, 0.5, 'orange'), # 45°-15° left
            ('front', 0, math.pi/6, 1.0, 'green'),             # 15° left to 15° right
            ('near_right', -math.pi/4, math.pi/6, 0.5, 'orange'), # 15°-45° right
            ('right', -math.pi/2, math.pi/4, 0.5, 'red')       # 45°-90° right
        ]
        
        # Threshold distances (in meters)
        self.us_cluster_threshold_front = 0.4  # Front ultrasonic
        self.us_cluster_threshold_side = 0.3   # Side ultrasonics
        self.rear_us_threshold = 0.8           # Rear ultrasonic
        self.lidar_threshold = 0.5             # LIDAR threshold
        
        # ROS interfaces
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.ultrasonic_cluster_sub = self.create_subscription(
            Float32MultiArray, '/sensors/ultrasonic_cluster_1', 
            self.ultrasonic_cluster_callback, ultrasonic_qos)
        self.rear_ultrasonic_sub = self.create_subscription(
            Float32MultiArray, '/sensors/ultrasonic_cluster_2', 
            self.rear_ultrasonic_callback, ultrasonic_qos)
        
        self.haptic_pub = self.create_publisher(
            Int8MultiArray, '/haptic', 10)
        
        # Sensor data storage
        self.lidar_data = None
        self.ultrasonic_cluster_data = [float('inf')] * 3  # [left, front, right]
        self.rear_ultrasonic_data = float('inf')
        self.detection_result = [0] * 5  # For [a,b,c,d,e] feedback
        
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
        
        # Create LIDAR sector visualizations
        self.lidar_sector_artists = []
        for name, center, width, threshold, color in self.lidar_sectors:
            wedge = Wedge(
                (0,0), threshold, 
                np.degrees(center-width/2), 
                np.degrees(center+width/2),
                color=color, alpha=0.2
            )
            self.lidar_sector_artists.append(self.ax.add_patch(wedge))
            self.ax.text(center, threshold*0.8, name, ha='center')
        
        # Ultrasonic positions and detection markers
        self.us_markers = []
        colors = ['red', 'green', 'blue', 'magenta']
        for i in range(3):  # Front cluster sensors
            marker, = self.ax.plot([], [], 'o', color=colors[i], markersize=10)
            self.us_markers.append(marker)
        
        # Rear sensor marker (shown at its physical position but facing forward)
        self.rear_marker, = self.ax.plot([math.pi], [0.5], 'o', color='magenta', markersize=12)
        self.rear_beam, = self.ax.plot([0, 0], [0.5, 2.0], 'm--', alpha=0.5)
        
        # Detection indicators (5 circles for [a,b,c,d,e])
        self.detection_artists = []
        # Left sector indicators (a)
        self.detection_artists.append(self.ax.add_patch(Circle((math.pi/2, 0.15), 0.05, color='red', alpha=0)))
        # Near left indicator (b)
        self.detection_artists.append(self.ax.add_patch(Circle((math.pi/4, 0.15), 0.05, color='orange', alpha=0)))
        # Front indicator (c)
        self.detection_artists.append(self.ax.add_patch(Circle((0, 0.2), 0.05, color='green', alpha=0)))
        # Near right indicator (d)
        self.detection_artists.append(self.ax.add_patch(Circle((-math.pi/4, 0.15), 0.05, color='orange', alpha=0)))
        # Right indicator (e)
        self.detection_artists.append(self.ax.add_patch(Circle((-math.pi/2, 0.15), 0.05, color='red', alpha=0)))
        
        # Add legend
        self.ax.legend(
            handles=[
                plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=10, label='Left'),
                plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='orange', markersize=10, label='Near Left/Right'),
                plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='green', markersize=10, label='Front'),
                plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', markersize=10, label='Ultrasonic'),
                plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='magenta', markersize=10, label='Rear US')
            ],
            loc='upper right'
        )
        
        self.fig.tight_layout()
        self.fig.canvas.draw()

    def lidar_callback(self, msg):
        """Store LIDAR data"""
        self.lidar_data = msg

    def ultrasonic_cluster_callback(self, msg):
        """Store ultrasonic cluster data [left, front, right]"""
        if len(msg.data) == 3:
            self.ultrasonic_cluster_data = msg.data

    def rear_ultrasonic_callback(self, msg):
        """Store rear ultrasonic sensor data"""
        if len(msg.data) == 1:
            self.rear_ultrasonic_data = msg.data[0]

    def process_lidar_data(self):
        """Process LIDAR data to detect obstacles in 5 sectors"""
        if self.lidar_data is None:
            return [0] * 5
        
        feedback = [0] * 5
        angles = np.linspace(
            self.lidar_data.angle_min,
            self.lidar_data.angle_max,
            len(self.lidar_data.ranges))
        
        ranges = np.array(self.lidar_data.ranges)
        
        for i, (_, center, width, threshold, _) in enumerate(self.lidar_sectors):
            start_angle = center - width/2
            end_angle = center + width/2
            
            # Handle angle wrapping
            if start_angle < -np.pi:
                start_angle += 2*np.pi
            if end_angle > np.pi:
                end_angle -= 2*np.pi
                
            if start_angle > end_angle:
                mask = (angles >= start_angle) | (angles <= end_angle)
            else:
                mask = (angles >= start_angle) & (angles <= end_angle)
            
            sector_ranges = ranges[mask]
            valid_ranges = sector_ranges[
                (sector_ranges > self.min_threshold) & 
                (sector_ranges <= threshold) & 
                ~np.isinf(sector_ranges)
            ]
            
            if valid_ranges.size > 0:
                feedback[i] = 1
                
        return feedback

    def process_ultrasonic_data(self):
        """Process ultrasonic data to detect obstacles"""
        feedback = [0] * 5
        us_left, us_front, us_right = self.ultrasonic_cluster_data
        rear_us = self.rear_ultrasonic_data
        
        # Map ultrasonic to LIDAR sectors
        # Left ultrasonic affects left and near-left sectors (a,b)
        if self.min_threshold <= us_left <= self.us_cluster_threshold_side:
            feedback[0] = 1  # left
            feedback[1] = 1  # near-left
            
        # Front ultrasonic affects front sector (c)
        if self.min_threshold <= us_front <= self.us_cluster_threshold_front:
            feedback[2] = 1  # front
            
        # Right ultrasonic affects right and near-right sectors (d,e)
        if self.min_threshold <= us_right <= self.us_cluster_threshold_side:
            feedback[3] = 1  # near-right
            feedback[4] = 1  # right
            
        # Rear ultrasonic affects front sector (c)
        if self.min_threshold <= rear_us <= self.rear_us_threshold:
            feedback[2] = 1  # front
            
        return feedback

    def update_detection(self):
        """Combine sensor data and update visualization"""
        try:
            # Process both sensor types
            lidar_feedback = self.process_lidar_data()
            ultrasonic_feedback = self.process_ultrasonic_data()
            
            # Combine detections (logical OR)
            combined_feedback = [int(l or u) for l, u in zip(lidar_feedback, ultrasonic_feedback)]
            
            # Update sensor positions in visualization
            for i, angle in enumerate(self.us_angles):
                distance = min(self.ultrasonic_cluster_data[i], 2.0)
                self.us_markers[i].set_data([angle], [distance])
            
            # Update rear sensor visualization
            self.rear_beam.set_data([0, 0], [0.5, min(self.rear_ultrasonic_data, 2.0)])
            
            # Update detection visualization
            for i, artist in enumerate(self.detection_artists):
                artist.set_alpha(combined_feedback[i] * 0.8)
            
            # Publish haptic feedback
            feedback_msg = Int8MultiArray()
            feedback_msg.data = combined_feedback
            self.haptic_pub.publish(feedback_msg)
            self.detection_result = combined_feedback
            
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