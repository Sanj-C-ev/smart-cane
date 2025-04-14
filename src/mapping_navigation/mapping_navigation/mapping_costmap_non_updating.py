import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
import tf_transformations as tf
import tf2_ros
import matplotlib.pyplot as plt
import scipy.ndimage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import os
import datetime

class MultiSensorSlam(Node):
    def __init__(self):
        super().__init__('multi_sensor_slam')
        
        # Subscribers
        ultrasonic_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/odom_rf2o', self.pose_callback, 10)
        self.create_subscription(Float32MultiArray, '/sensors/ultrasonic_cluster_1', self.ultrasonic_cluster1_callback, ultrasonic_qos)
        self.create_subscription(Float32MultiArray, '/sensors/ultrasonic_cluster_2', self.ultrasonic_cluster2_callback, ultrasonic_qos)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        
        # Publishers
        self.occupancy_grid_publisher = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)
        
        # Variables
        self.grid_size = 500
        self.resolution = 0.025  # 5 cm per cell
        self.occupancy_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        self.ultrasonic_log_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        self.lidar_log_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.sensor_distances_cluster1 = [0.5, 0.5, 0.5]
        self.sensor_distances_cluster2 = [0.5]
        self.lidar_ranges = []
        self.lidar_angles = []

        # Set the timer to publish the map at 2 Hz (every 0.5 seconds)
        self.create_timer(0.5, self.publish_occupancy_grid)
        
        # Variables for feedback and visualization
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.center_x = self.x
        self.center_y = self.y

    def pose_callback(self, msg):
        """Updates the current robot pose for map generation."""
        self.x, self.y = msg.pose.pose.position.x, msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = tf.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.generate_new_map()


    def ultrasonic_cluster1_callback(self, msg):
        """Handle ultrasonic sensor data from cluster 1."""
        if len(msg.data) == 3:
            self.sensor_distances_cluster1 = msg.data
        self.generate_new_map()

    def ultrasonic_cluster2_callback(self, msg):
        """Handle ultrasonic sensor data from cluster 2."""
        if len(msg.data) == 1:
            self.sensor_distances_cluster2 = msg.data
        self.generate_new_map()


    def lidar_callback(self, msg):
        """Handle LiDAR sensor data."""
        self.lidar_ranges = np.array(msg.ranges)
        self.lidar_angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        self.generate_new_map()

    def generate_new_map(self):
        """Create a new map based on the current sensor data."""
        # Reset map and log grids
        self.occupancy_grid.fill(0)
        self.ultrasonic_log_grid.fill(0)
        self.lidar_log_grid.fill(0)

        # Generate new sensor points
        ultrasonic_points = self.generate_ultrasonic_arcs(self.sensor_distances_cluster1, 0.707, 0.0, [-np.pi / 4, 0, np.pi / 4])
        lidar_points = self.transform_lidar_points()

        # Update log grids based on sensor data
        self.update_occupancy_grid(ultrasonic_points, self.ultrasonic_log_grid, 0.5)
        self.update_occupancy_grid(lidar_points, self.lidar_log_grid, 0.8)
        self.fuse_maps()

    def generate_ultrasonic_arcs(self, sensor_distances, cluster_offset_x, cluster_offset_y, angles):
        """Generate points for ultrasonic sensor arcs."""
        arc_points = []
        beam_width = np.radians(15)  # Typical ultrasonic beam width
        
        cluster_x = self.x + cluster_offset_x * np.cos(self.yaw) - cluster_offset_y * np.sin(self.yaw)
        cluster_y = self.y + cluster_offset_x * np.sin(self.yaw) + cluster_offset_y * np.cos(self.yaw)
        
        for i, dist in enumerate(sensor_distances):
            if 0.02 < dist < 3.0:  # Valid range
                sensor_angle = angles[i]
                arc_res = int(beam_width / np.radians(2))  # 2-degree resolution
                arc_angles = np.linspace(-beam_width / 2, beam_width / 2, arc_res)
                
                for theta in arc_angles:
                    effective_angle = self.yaw + sensor_angle + theta
                    world_x = cluster_x + dist * np.cos(effective_angle)
                    world_y = cluster_y + dist * np.sin(effective_angle)
                    arc_points.append((world_x, world_y))
        return arc_points

    def transform_lidar_points(self):
        """Transform LiDAR points to the global frame."""
        points = []
        for r, a in zip(self.lidar_ranges, self.lidar_angles):
            if 0.1 < r < 10.0:
                world_x = self.x + r * np.cos(self.yaw + a)
                world_y = self.y + r * np.sin(self.yaw + a)
                points.append((world_x, world_y))
        return points

    def update_occupancy_grid(self, sensor_points, log_grid, weight):
        """Update the occupancy grid based on sensor points."""
        robot_grid_x, robot_grid_y = self.world_to_grid(self.x, self.y)
        for (x, y) in sensor_points:
            grid_x, grid_y = self.world_to_grid(x, y)
            if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                log_grid[grid_y, grid_x] += weight * self.l_occ
                free_cells = self.bresenham(robot_grid_x, robot_grid_y, grid_x, grid_y)
                for fx, fy in free_cells:
                    if 0 <= fx < self.grid_size and 0 <= fy < self.grid_size:
                        log_grid[fy, fx] += self.l_free

    def bresenham(self, x0, y0, x1, y1):
        """Bresenham's line algorithm to generate a list of points from (x0, y0) to (x1, y1)."""
        points = []
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx, sy = 1 if x0 < x1 else -1, 1 if y0 < y1 else -1
        err = dx - dy
        while (x0, y0) != (x1, y1):
            points.append((x0, y0))
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return points

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates."""
        grid_x = int((x / self.resolution) + self.grid_size // 2)
        grid_y = int((y / self.resolution) + self.grid_size // 2)
        return grid_x, grid_y

    def fuse_maps(self):
        """Fuse ultrasonic and lidar maps into a single occupancy grid."""
        ultrasonic_prob = 1 / (1 + np.exp(-self.ultrasonic_log_grid))
        lidar_prob = 1 / (1 + np.exp(-self.lidar_log_grid))
        self.occupancy_grid = np.maximum(ultrasonic_prob, lidar_prob) * 100

    def publish_occupancy_grid(self):
        """Publish the generated occupancy grid at 2 Hz."""
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "map"
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.grid_size
        grid_msg.info.height = self.grid_size
        grid_msg.info.origin.position.x = -self.grid_size * self.resolution / 2
        grid_msg.info.origin.position.y = -self.grid_size * self.resolution / 2
        grid_msg.data = self.occupancy_grid.flatten().astype(np.int8).tolist()
        self.occupancy_grid_publisher.publish(grid_msg)
        self.get_logger().info("Published new occupancy grid.")

def main():
    rclpy.init()
    node = MultiSensorSlam()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
