import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
import tf_transformations as tf
import tf2_ros
import matplotlib.pyplot as plt
import scipy.ndimage


class MultiSensorSlam(Node):
    def __init__(self):
        super().__init__('multi_sensor_slam')

        self.create_subscription(Odometry, '/odom_rf2o', self.odom_callback, 10)
        self.create_subscription(Float32MultiArray, '/ultrasonic_cluster_1', self.ultrasonic_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.grid_size = 500
        self.resolution = 0.025
        self.occupancy_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        self.local_costmap = np.zeros_like(self.occupancy_grid)
        self.p_hit = 0.7
        self.l_occ = np.log(self.p_hit / (1 - self.p_hit))
        self.l_free = np.log((1 - self.p_hit) / self.p_hit)
        self.ultrasonic_log_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        self.lidar_log_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.sensor_distances = [0.5, 0.5, 0.5]
        self.lidar_ranges = []
        self.lidar_angles = []
        self.ultrasonic_arc_points = []

        self.occupancy_grid_publisher = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)
        self.local_costmap_publisher = self.create_publisher(OccupancyGrid, '/local_costmap', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/robot_pose', 10)

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.create_timer(0.1, self.update_maps)
        self.create_timer(0.1, self.publish_occupancy_grid)

        self.center_x = self.x
        self.center_y = self.y

    def odom_callback(self, msg):
        self.x, self.y = msg.pose.pose.position.x, msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = tf.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.check_and_shift_map()

    def ultrasonic_callback(self, msg):
        if len(msg.data) == 3:
            self.sensor_distances = msg.data
        self.ultrasonic_arc_points = self.generate_ultrasonic_arcs()

    def lidar_callback(self, msg):
        self.lidar_ranges = np.array(msg.ranges)
        self.lidar_angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

    def generate_ultrasonic_arcs(self):
        arc_points = []
        angles = [np.pi / 4, 0, -np.pi / 4]
        arc_half_angle = np.radians(7.5)
        cluster_x = self.x
        cluster_y = self.y

        for i, dist in enumerate(self.sensor_distances):
            if 0.02 < dist < 2.0:
                sensor_angle = self.yaw - angles[i]
                arc_res = 10
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
        grid_x = int((x / self.resolution) + self.grid_size // 2)
        grid_y = int((y / self.resolution) + self.grid_size // 2)
        return grid_x, grid_y

    def update_occupancy_grid(self, sensor_points, log_grid, weight):
        robot_grid_x, robot_grid_y = self.world_to_grid(self.x, self.y)
        for (x, y) in sensor_points:
            grid_x, grid_y = self.world_to_grid(x, y)
            if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                log_grid[grid_y, grid_x] += weight * self.l_occ
                free_cells = self.bresenham(robot_grid_x, robot_grid_y, grid_x, grid_y)
                for fx, fy in free_cells:
                    if 0 <= fx < self.grid_size and 0 <= fy < self.grid_size:
                        log_grid[fy, fx] += self.l_free

    def update_local_costmap(self):
        inflation_radius_m = 0.2
        inflation_radius = int(inflation_radius_m / self.resolution)
        self.local_costmap = np.zeros_like(self.occupancy_grid)
        obstacle_mask = self.occupancy_grid > 50
        distance_map = scipy.ndimage.distance_transform_edt(~obstacle_mask) * self.resolution
        inflated_costmap = 100 * np.exp(-((distance_map / inflation_radius_m) ** 2))
        self.local_costmap = np.where(distance_map <= inflation_radius_m, inflated_costmap, 0).astype(np.uint8)

    def update_maps(self):
        # Update log-odds maps with current sensor readings
        self.update_occupancy_grid(self.ultrasonic_arc_points, self.ultrasonic_log_grid, 0.5)
        self.update_occupancy_grid(self.transform_lidar_points(), self.lidar_log_grid, 0.8)

        # Apply exponential smoothing to reduce noise
        self.smooth_log_grids(alpha=0.6)

        # Fuse smoothed maps into occupancy grid
        self.fuse_maps()

        # Generate inflated local costmap
        self.update_local_costmap()

    def smooth_log_grids(self, alpha=0.6):
        """
        Smooth log-odds grids using Exponential Moving Average (EMA).
        alpha: smoothing factor (0 < alpha < 1), lower = more smoothing
        """
        # Store previous values if not initialized
        if not hasattr(self, 'prev_ultrasonic_log_grid'):
            self.prev_ultrasonic_log_grid = np.copy(self.ultrasonic_log_grid)
            self.prev_lidar_log_grid = np.copy(self.lidar_log_grid)

        # Apply EMA
        self.ultrasonic_log_grid = alpha * self.ultrasonic_log_grid + (1 - alpha) * self.prev_ultrasonic_log_grid
        self.lidar_log_grid = alpha * self.lidar_log_grid + (1 - alpha) * self.prev_lidar_log_grid

        # Update previous for next iteration
        self.prev_ultrasonic_log_grid = np.copy(self.ultrasonic_log_grid)
        self.prev_lidar_log_grid = np.copy(self.lidar_log_grid)


    def fuse_maps(self):
        ultrasonic_prob = 1 / (1 + np.exp(-self.ultrasonic_log_grid))
        ultrasonic_binary = ultrasonic_prob > 0.5
        lidar_prob = 1 / (1 + np.exp(-self.lidar_log_grid))
        fused_prob = np.maximum(ultrasonic_binary, lidar_prob)
        self.occupancy_grid = fused_prob * 100

    def check_and_shift_map(self):
        threshold = 2.5
        if abs(self.x - self.center_x) > threshold or abs(self.y - self.center_y) > threshold:
            self.shift_map()

    def shift_map(self):
        self.center_x = self.x
        self.center_y = self.y
        self.occupancy_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        self.ultrasonic_log_grid.fill(0)
        self.lidar_log_grid.fill(0)

    def publish_occupancy_grid(self):
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

        costmap_msg = OccupancyGrid()
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        costmap_msg.header.frame_id = "map"
        costmap_msg.info.resolution = self.resolution
        costmap_msg.info.width = self.grid_size
        costmap_msg.info.height = self.grid_size
        costmap_msg.info.origin.position.x = -self.grid_size * self.resolution / 2
        costmap_msg.info.origin.position.y = -self.grid_size * self.resolution / 2
        costmap_msg.data = self.local_costmap.flatten().tolist()
        self.local_costmap_publisher.publish(costmap_msg)

        self.publish_map_transform()

        pose_msg = PoseStamped()
        quaternion = tf.quaternion_from_euler(0, 0, self.yaw)
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
        self.pose_publisher.publish(pose_msg)

    def publish_map_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        q = tf.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def render_map(self):
        self.ax.cla()
        gx, gy = self.world_to_grid(self.x, self.y)
        self.ax.imshow(self.occupancy_grid, cmap='gray')
        self.ax.plot(gx, gy, marker='x', color='r')
        plt.draw()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    slam_node = MultiSensorSlam()
    rclpy.spin(slam_node)
    slam_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
