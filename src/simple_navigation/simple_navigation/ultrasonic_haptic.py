#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8MultiArray
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, Circle
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class UltrasonicObstacleVisualizer(Node):
    def __init__(self):
        super().__init__('ultrasonic_obstacle_visualizer')
        ultrasonic_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        # Sensor configuration
        #self.front_cluster_offset = 0.5  # 45cm ahead of the rear sensor
        #self.rear_sensor_offset = -0.5   # 45cm behind the front cluster
        self.min_threshold = 0.05
        
        # Ultrasonic sensor angles (45° left, 0° front, 45° right)
        self.us_angles = [math.pi/4, 0, -math.pi/4]
        self.rear_us_angle = 0  # Also facing forward
        
        # Sector configuration
        self.sectors = [
            ('left', math.pi/4, math.pi/4, 0.3, 'red'),     # 45° left
            ('front', 0, math.pi/6, 0.3, 'green'),          # Center
            ('right', -math.pi/4, math.pi/4, 0.3, 'red')    # 45° right
        ]
        
        # Threshold distances (in meters)
        self.us_cluster_threshold_front = 0.4  # 30cm for front cluster
        self.us_cluster_threshold_side = 0.3  # 30cm for front cluster

        self.rear_us_threshold = 0.8     # 50cm for rear sensor
        
        # ROS interfaces
        self.ultrasonic_cluster_sub = self.create_subscription(
            Float32MultiArray, '/sensors/ultrasonic_cluster_1', 
            self.ultrasonic_cluster_callback, ultrasonic_qos)
        self.rear_ultrasonic_sub = self.create_subscription(
            Float32MultiArray, '/sensors/ultrasonic_cluster_2', 
            self.rear_ultrasonic_callback, ultrasonic_qos)
        
        self.haptic_pub = self.create_publisher(
            Int8MultiArray, '/haptic', 10)
        
        # Sensor data storage
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
        self.ax.set_title("Ultrasonic Obstacle Detection\n(Red=Obstacle)")
        
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
        
        # Ultrasonic positions and detection markers
        self.us_markers = []
        colors = ['red', 'green', 'blue', 'magenta']
        for i in range(3):  # Front cluster sensors
            marker, = self.ax.plot([], [], 'o', color=colors[i], markersize=10)
            self.us_markers.append(marker)
        
        # Rear sensor marker (shown at its physical position but facing forward)
        self.rear_marker, = self.ax.plot([math.pi], [0.45], 'o', color='magenta', markersize=12)
        self.rear_beam, = self.ax.plot([0, 0], [0.45, 2.0], 'm--', alpha=0.5)  # Show detection beam
        
        # Detection indicators (now 5 circles for [a,b,c,d,e])
        self.detection_artists = []
        # Left sector indicators (a,b)
        self.detection_artists.append(self.ax.add_patch(Circle((math.pi/4, 0.15), 0.05, color='red', alpha=0)))
        self.detection_artists.append(self.ax.add_patch(Circle((math.pi/4, 0.25), 0.05, color='red', alpha=0)))
        # Front sector indicator (c)
        self.detection_artists.append(self.ax.add_patch(Circle((0, 0.2), 0.05, color='green', alpha=0)))
        # Right sector indicators (d,e)
        self.detection_artists.append(self.ax.add_patch(Circle((-math.pi/4, 0.15), 0.05, color='red', alpha=0)))
        self.detection_artists.append(self.ax.add_patch(Circle((-math.pi/4, 0.25), 0.05, color='red', alpha=0)))
        
        # Add legend
        self.ax.legend(
            handles=[
                plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=10, label='Left US'),
                plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='green', markersize=10, label='Front US'),
                plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', markersize=10, label='Right US'),
                plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='magenta', markersize=10, label='Rear US (fwd)')
            ],
            loc='upper right'
        )
        
        self.fig.tight_layout()
        self.fig.canvas.draw()

    def ultrasonic_cluster_callback(self, msg):
        """Store 3-sensor ultrasonic cluster data [left, front, right]"""
        if len(msg.data) == 3:
            self.ultrasonic_cluster_data = msg.data
            self.get_logger().info(f"Cluster data: {msg.data}")

    def rear_ultrasonic_callback(self, msg):
        """Store rear ultrasonic sensor data"""
        if len(msg.data) == 1:
            self.rear_ultrasonic_data = msg.data[0]
            self.get_logger().info(f"Rear sensor: {msg.data[0]}")

    def update_detection(self):
        """Process sensor data and update visualization"""
        try:
            feedback = [0] * 5  # Initialize [a,b,c,d,e]
            
            # Get ultrasonic data
            us_left, us_front, us_right = self.ultrasonic_cluster_data
            rear_us = self.rear_ultrasonic_data
            
            # Update sensor positions in visualization
            for i, angle in enumerate(self.us_angles):
                distance = min(self.ultrasonic_cluster_data[i], 2.0)
                self.us_markers[i].set_data([angle], [distance])
            
            # Update rear sensor visualization
            self.rear_beam.set_data([0, 0], [0.45, min(rear_us, 2.0)])
            
            # Left sector detection (a,b)
            if self.min_threshold <= us_left <= self.us_cluster_threshold_side:
                feedback[0] = 1  # a
                feedback[1] = 1  # b
            
            # Front sector detection (c)
            if self.min_threshold <= us_front <= self.us_cluster_threshold_front or self.min_threshold <= rear_us <= self.rear_us_threshold:
                feedback[2] = 1  # c
            
            # Right sector detection (d,e)
            if self.min_threshold <= us_right <= self.us_cluster_threshold_side:
                feedback[3] = 1  # d
                feedback[4] = 1  # e
            
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
    node = UltrasonicObstacleVisualizer()
    
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