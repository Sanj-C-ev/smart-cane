import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import tf_transformations as tf
from matplotlib.patches import Arrow
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import Int32MultiArray
import networkx as nxx  # Graph library


class MultiSensorSlam(Node):
    def __init__(self):
        super().__init__('multi_sensor_slam')

        # Subscribe to topics
        self.create_subscription(Odometry, '/odom_rf2o', self.odom_callback, 10)
        self.create_subscription(Float32MultiArray, '/ultrasonic_cluster_1', self.ultrasonic_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.graph_pub = self.create_publisher(Int32MultiArray, '/graph_map', 10)
        self.odom_pub = self.create_publisher(Float32MultiArray, '/odom_info', 10)  # New publisher for odometry
        self.goal_pub = self.create_publisher(Float32MultiArray, '/goal_position', 10)  # Publisher for goal

        # Timer to Publish Graph Periodically
        self.create_timer(1.0, self.publish_graph)
        
        # Initialize occupancy grid parameters
        self.grid_size = 200  # 200x200 grid
        self.resolution = 0.05  # 5 cm per cell
        self.occupancy_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        self.ultrasonic_log_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        self.lidar_log_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        # Log-odds parameters for mapping
        self.p_hit = 0.7
        self.l_occ = np.log(self.p_hit / (1 - self.p_hit))
        self.l_free = np.log((1 - self.p_hit) / self.p_hit)

        # Robot pose initialization
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0

        # Sensor data initialization
        self.sensor_distances = [0.5, 0.5, 0.5]
        self.lidar_ranges = []
        self.lidar_angles = []
        self.ultrasonic_arc_points = []

        # Separate log-odds grids for ultrasonic and LiDAR
        self.ultrasonic_log_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        self.lidar_log_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)

        # Initialize Matplotlib for visualization
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 6))

        # Timers for updates
        self.ultrasonic_timer = self.create_timer(0.1, self.update_ultrasonic_map)  # 10 Hz
        self.lidar_timer = self.create_timer(0.5, self.update_lidar_map)  # 2 Hz

        # Goal initialization
        self.goal_x, self.goal_y = None, None

    def odom_callback(self, msg):
        # Update robot pose from odometry
        self.x, self.y = msg.pose.pose.position.x, msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = tf.euler_from_quaternion([q.x, q.y, q.z, q.w])

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

        for i, dist in enumerate(self.sensor_distances):
            if 0.02 < dist < 4.99:
                sensor_angle = self.yaw - angles[i]
                arc_angles = np.linspace(sensor_angle - arc_half_angle, sensor_angle + arc_half_angle, 10)

                for theta in arc_angles:
                    world_x = self.x + dist * np.cos(theta)
                    world_y = self.y + dist * np.sin(theta)
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
                log_grid[grid_x, grid_y] += weight * self.l_occ  # Fix: Swap X and Y
                free_cells = self.bresenham(robot_grid_x, robot_grid_y, grid_x, grid_y)
                for fx, fy in free_cells:
                    if 0 <= fx < self.grid_size and 0 <= fy < self.grid_size:
                        log_grid[fy, fx] += self.l_free  # Fix: Swap X and Y

    def update_ultrasonic_map(self):
        self.update_occupancy_grid(self.ultrasonic_arc_points, self.ultrasonic_log_grid, 0.5)
        self.fuse_maps()

    def update_lidar_map(self):
        sensor_points = self.transform_lidar_points()
        self.update_occupancy_grid(sensor_points, self.lidar_log_grid, 0.8)
        self.fuse_maps()

    def fuse_maps(self):
        ultrasonic_prob = 1 / (1 + np.exp(-self.ultrasonic_log_grid))
        ultrasonic_binary = ultrasonic_prob > 0.5 
        lidar_prob = 1 / (1 + np.exp(-self.lidar_log_grid))
        
        fused_prob = np.maximum(ultrasonic_binary, lidar_prob)
        self.occupancy_grid = fused_prob

    def render_map(self):
        self.fig.clf()
        ax = self.fig.add_subplot(1, 1, 1)
        ax.imshow(1 - self.occupancy_grid.T, cmap='gray', origin='lower')  # Fix: Transpose and set origin

        # Draw robot position
        robot_x, robot_y = self.world_to_grid(self.x, self.y)
        ax.add_patch(Arrow(robot_x, robot_y, 5 * np.cos(self.yaw), 5 * np.sin(self.yaw), width=2, color='red'))

        plt.draw()
        plt.pause(0.01)

    def bresenham(self, x0, y0, x1, y1):
        points = []
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
        err = dx - dy

        while (x0, y0) != (x1, y1):
            points.append((y0, x0))  # Fix: Swap X and Y
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return points

    def occupancy_to_graph(self):
        """ Convert occupancy grid to a graph representation using networkx. """
        graph = nxx.Graph()  # Create an empty graph using networkx
        threshold = 0.3  # Cells above this probability are obstacles

        for x in range(self.grid_size):
            for y in range(self.grid_size):
                # If the cell is free (below threshold)
                if self.occupancy_grid[x, y] < threshold:  
                    node_id = self.get_node_id(x, y)
                    graph.add_node(node_id)  # Add the node to the graph
                    
                    # Connect to 8 neighboring cells (if they are free)
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            if dx == 0 and dy == 0:
                                continue
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                                if self.occupancy_grid[nx, ny] < threshold:
                                    neighbor_id = self.get_node_id(nx, ny)
                                    graph.add_edge(node_id, neighbor_id)  # Add an edge between nodes

        return graph


    def get_node_id(self, x, y):
        """ Convert grid (x, y) to a unique node ID. """
        return x * self.grid_size + y

    def publish_graph(self):
        """ Publish the graph as a list of nodes and edges along with odometry. """
        graph = self.occupancy_to_graph()
        msg = Int32MultiArray()

        for node in graph.nodes:
            for neighbor in graph.neighbors(node):
                msg.data.append(node)
                msg.data.append(neighbor)

        self.graph_pub.publish(msg)

        # Create and publish odometry message with yaw
        odom_msg = Float32MultiArray()
        odom_msg.data = [self.x, self.y, self.yaw]  # Publish x, y, yaw
        self.odom_pub.publish(odom_msg)

        # If goal is set, publish goal information
        if self.goal_x is not None and self.goal_y is not None:
            goal_msg = Float32MultiArray()
            goal_msg.data = [self.goal_x, self.goal_y]
            self.goal_pub.publish(goal_msg)

        self.get_logger().info("Published graph and odometry information.")

    def set_goal(self, x, y):
        """ Method to set the goal position. """
        self.goal_x, self.goal_y = x, y


def main():
    rclpy.init()
    node = MultiSensorSlam()
    try:
        # Example of setting a goal position
        node.set_goal(110.0, 110.0)  # Set the goal at coordinates (5.0, 5.0)
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()

if __name__ == '__main__':
    main()
