import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf_transformations as tf
import matplotlib.pyplot as plt
from geometry_msgs.msg import Quaternion
import tf2_ros

class MultiSensorSlam(Node):
    def __init__(self):
        super().__init__('multi_sensor_slam')

        # Subscribe to topics
        self.create_subscription(Odometry, '/odom_rf2o', self.odom_callback, 10)
        self.create_subscription(Float32MultiArray, '/ultrasonic_cluster_1', self.ultrasonic_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # Initialize map parameters
        self.grid_size = 500  # 200x200 grid
        self.resolution = 0.025  # 5 cm per cell
        self.occupancy_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        self.p_hit = 0.7
        self.l_occ = np.log(self.p_hit / (1 - self.p_hit))
        self.l_free = np.log((1 - self.p_hit) / self.p_hit)

        # Initialize robot pose
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.sensor_distances = [0.5, 0.5, 0.5]
        self.lidar_ranges = []
        self.lidar_angles = []
        self.ultrasonic_arc_points = []
        # Separate log-odds grids for ultrasonic and LiDAR
        self.ultrasonic_log_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        self.lidar_log_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)

        # Initialize ROS publishers
        self.occupancy_grid_publisher = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/robot_pose', 10)

        # Initialize Matplotlib
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.create_timer(0.1, self.update_maps)  # Timer for updates
        self.create_timer(0.1, self.publish_occupancy_grid)  # Publish the grid at 1 Hz

        self.center_x = self.x
        self.center_y = self.y

    def odom_callback(self, msg):
        self.x, self.y = msg.pose.pose.position.x, msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = tf.euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.check_and_shift_map()

    def ultrasonic_callback(self, msg):
        if len(msg.data) == 3:
            self.sensor_distances = msg.data  # Right, Front, Left
        self.ultrasonic_arc_points = self.generate_ultrasonic_arcs()

    def lidar_callback(self, msg):
        self.lidar_ranges = np.array(msg.ranges)
        self.lidar_angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

    def generate_ultrasonic_arcs(self):
        arc_points = []
        angles = [np.pi / 4, 0, -np.pi / 4]  # Right, Front, Left
        arc_half_angle = np.radians(7.5)  # Half of 15 degrees

        cluster_x = self.x  # + 0.1 * np.cos(self.yaw)
        cluster_y = self.y  # + 0.1 * np.sin(self.yaw)

        for i, dist in enumerate(self.sensor_distances):
            if 0.02 < dist < 0.75:
                sensor_angle = self.yaw - angles[i]
                arc_res = 10  # Number of points per arc
                arc_angles = np.linspace(sensor_angle - arc_half_angle, sensor_angle + arc_half_angle, arc_res)

                for theta in arc_angles:
                    world_x = cluster_x + dist * np.cos(theta)
                    world_y = cluster_y + dist * np.sin(theta)
                    arc_points.append((world_x, world_y))
        return arc_points

    def transform_lidar_points(self):
        points = []
        for r, a in zip(self.lidar_ranges, self.lidar_angles):
            if 0.1 < r < 10.0:
                world_x = self.x + r * np.cos(self.yaw + a)
                world_y = self.y + r * np.sin(self.yaw + a)
                points.append((world_x, world_y))
        return points

    def bresenham(self, x0, y0, x1, y1):
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
        """ Convert world coordinates to grid indices """
        grid_x = int((x / self.resolution) + self.grid_size // 2)
        grid_y = int((y / self.resolution) + self.grid_size // 2)  # Flip Y-axis
        return grid_x, grid_y

    def update_occupancy_grid(self, sensor_points, log_grid, weight):
        robot_grid_x, robot_grid_y = self.world_to_grid(self.x, self.y)

        for (x, y) in sensor_points:
            grid_x, grid_y = self.world_to_grid(x, y)

            if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                log_grid[grid_y, grid_x] += weight * self.l_occ  # Fix: Swap X and Y
                free_cells = self.bresenham(robot_grid_x, robot_grid_y, grid_x, grid_y)
                for fx, fy in free_cells:
                    if 0 <= fx < self.grid_size and 0 <= fy < self.grid_size:
                        log_grid[fy, fx] += self.l_free  # Fix: Swap X and Y


    def update_maps(self):
        # Update the occupancy grid using ultrasonic and lidar data
        self.update_occupancy_grid(self.ultrasonic_arc_points, self.ultrasonic_log_grid, 0.5)
        self.update_occupancy_grid(self.transform_lidar_points(), self.lidar_log_grid, 0.8)
        self.fuse_maps()

        # Render the map to visualize
        #self.render_map()

    def fuse_maps(self):
        ultrasonic_prob = 1 / (1 + np.exp(-self.ultrasonic_log_grid))
        ultrasonic_binary = ultrasonic_prob > 0.5
        lidar_prob = 1 / (1 + np.exp(-self.lidar_log_grid))

        fused_prob = np.maximum(ultrasonic_binary, lidar_prob)
        self.occupancy_grid = fused_prob

    def check_and_shift_map(self):
        # Check if the robot has moved too far away from the current map center
        threshold = 2.5  # 5 meters
        if abs(self.x - self.center_x) > threshold or abs(self.y - self.center_y) > threshold:
            self.shift_map()

    def shift_map(self):
        # Shift the map such that the robot is at the new center
        self.center_x = self.x
        self.center_y = self.y

        # Create a new map centered on the robot's position
        self.occupancy_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)

    def publish_occupancy_grid(self):
        # Create and publish the Occupancy Grid message
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "map"

        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.grid_size
        grid_msg.info.height = self.grid_size
        grid_msg.info.origin.position.x = -self.grid_size * self.resolution / 2
        grid_msg.info.origin.position.y = -self.grid_size * self.resolution / 2
        grid_msg.data = (self.occupancy_grid.flatten() * 100).astype(np.int8).tolist()  # Occupancy grid values are in range [0, 100]

        # Publish the occupancy grid
        self.occupancy_grid_publisher.publish(grid_msg)
        self.publish_map_transform()
        # Publish the robot's pose
        pose_msg = PoseStamped()
        quaternion = tf.quaternion_from_euler(0, 0, self.yaw)

        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.orientation = Quaternion(
                                                    x=quaternion[0], 
                                                    y=quaternion[1], 
                                                    z=quaternion[2], 
                                                    w=quaternion[3]
                                                )
        self.pose_publisher.publish(pose_msg)

    def publish_map_transform(self):
        # Broadcasting a transform from odom frame to map frame
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'  # Define the parent frame
        transform.child_frame_id = 'occupancy_grid'  # Define the child frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)
        
    def render_map(self):
        self.ax.cla()  # Clear the current axes
        ax1 = self.fig.add_subplot(1, 3, 1)
        ax2 = self.fig.add_subplot(1, 3, 2)
        ax3 = self.fig.add_subplot(1, 3, 3)

        ultrasonic_map = 1 / (1 + np.exp(-self.ultrasonic_log_grid))  # Convert log-odds to probability
        lidar_map = 1 / (1 + np.exp(-self.lidar_log_grid))  # Convert log-odds to probability
        fused_map = self.occupancy_grid
        ax1.imshow(1 - ultrasonic_map, cmap='gray', origin='upper')
        ax1.set_title("Ultrasonic Map")

        ax2.imshow(1 - lidar_map, cmap='gray', origin='upper')
        ax2.set_title("LiDAR Map")

        ax3.imshow(1 - fused_map, cmap='gray', origin='upper')
        ax3.set_title("Fused Map")

        plt.draw()
        plt.pause(0.01)

def main():
    rclpy.init()
    node = MultiSensorSlam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
