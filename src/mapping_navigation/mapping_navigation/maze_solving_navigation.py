import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from queue import PriorityQueue
import tf_transformations as tf


class CustomNavigation(Node):
    def __init__(self):
        super().__init__('custom_navigation')
        
        # Subscribers
        self.create_subscription(OccupancyGrid, '/occupancy_grid', self.map_callback, 10)
        self.create_subscription(OccupancyGrid, '/local_costmap', self.costmap_callback, 10)
        self.create_subscription(PoseStamped, '/next_waypoint', self.waypoint_callback, 10)
        self.create_subscription(PoseStamped, '/smart_cane_pose', self.pose_callback, 10)  # New subscriber
        
        # Publishers
        self.goal_publisher = self.create_publisher(PoseStamped, '/smart_cane_goal', 10)
        self.path_publisher = self.create_publisher(Path, '/smart_cane_path', 10)
    
        # Variables
        self.occupancy_grid = None
        self.resolution = 0.025  # Assuming 5 cm per cell
        self.width = 500  # Grid width
        self.height = 500  # Grid height
        self.origin_x = -5.0  # Map origin (meters)
        self.origin_y = -5.0
        self.costmap = None
        self.global_goal = (1.0, 0.0)  # Default goal (updated via waypoint)
        self.current_pose = None  # Stores the latest robot pose

    def pose_callback(self, msg):
        """Updates the current robot pose for path planning."""
        self.current_pose = msg
        self.try_planning()

    def map_callback(self, msg):
        self.occupancy_grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.try_planning()

    def waypoint_callback(self, msg):
        self.global_goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"Updated global goal to: {self.global_goal}")
        self.try_planning()   

    def costmap_callback(self, msg):
        self.costmap = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.try_planning()

    def try_planning(self):
        """Attempts path planning if all required data is available."""
        if (
            self.occupancy_grid is None 
            or self.costmap is None 
            or self.current_pose is None
        ):
            return

        # Search for path to the goal in front of the robot
        if not self.find_path_in_front():
            self.get_logger().info("No valid path in front, attempting to rotate and search.")
            # If no path found, rotate and check the new direction (90, 180, 270 degrees)
            if not self.rotate_and_find_path(90):
                if not self.rotate_and_find_path(180):
                    if not self.rotate_and_find_path(270):
                        self.get_logger().info("No path found after 3 rotations.")
                        return  # No path found after trying all directions

    def find_path_in_front(self):
        """Search for a goal within a 1-1.5m square region in front of the robot."""
        goal_x_range = (1.0, 1.5)
        goal_y_range = (-0.25, 0.25)
        
        for goal_x in np.linspace(goal_x_range[0], goal_x_range[1], num=5):  # Sample goals along x direction
            for goal_y in np.linspace(goal_y_range[0], goal_y_range[1], num=5):  # Sample goals along y direction
                goal = (goal_x, goal_y)
                self.get_logger().info(f"Searching for path to goal: {goal}")
                if self.plan_path(goal):
                    return True
        return False

    def rotate_and_find_path(self, angle):
        """Rotate the robot by the given angle and search for a path."""
        self.get_logger().info(f"Rotating by {angle} degrees and searching for a path.")
        # Rotate robot by `angle` degrees (convert to radians)
        self.current_pose.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, np.radians(angle))
        return self.find_path_in_front()

    def plan_path(self, target_goal):
        """Plans path from current pose to the target goal."""
        start_x = self.current_pose.pose.position.x
        start_y = self.current_pose.pose.position.y
        start = self.global_to_grid((start_x, start_y))  # Dynamic start position
        
        goal = self.global_to_grid(target_goal)
        path = self.d_lite(start, goal)
        
        if path:
            self.publish_path(path)
            self.publish_goal(target_goal)
            return True
        return False

    def d_lite(self, start, goal):
        frontier = PriorityQueue()
        frontier.put((0, start))
        came_from = {}
        cost_so_far = {start: 0}

        while not frontier.empty():
            _, current = frontier.get()
            if current == goal:
                break

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = current[0] + dx, current[1] + dy

                if 0 <= nx < self.width and 0 <= ny < self.height:
                    occ = self.occupancy_grid[ny, nx]
                    inflation = self.costmap[ny, nx]

                    # Skip if occupied (hard obstacle)
                    if occ >= 50:
                        continue

                    # Optional: discourage moving into unknown space
                    if occ == -1:
                        continue

                    inflation_cost = inflation / 100.0

                    move_cost = 1 + inflation_cost
                    new_cost = cost_so_far[current] + move_cost

                    next_cell = (nx, ny)
                    if next_cell not in cost_so_far or new_cost < cost_so_far[next_cell]:
                        cost_so_far[next_cell] = new_cost
                        priority = new_cost + np.hypot(goal[0] - nx, goal[1] - ny)
                        frontier.put((priority, next_cell))
                        came_from[next_cell] = current

        # Reconstruct path
        path = []
        current = goal
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path

    def global_to_grid(self, pos):
        gx, gy = pos
        x = int((gx - self.origin_x) / self.resolution)
        y = int((gy - self.origin_y) / self.resolution)
        return x, y
    
    def grid_to_global(self, grid_pos):
        x, y = grid_pos
        gx = x * self.resolution + self.origin_x
        gy = y * self.resolution + self.origin_y
        return gx, gy
    
    def publish_goal(self, goal):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = goal[0]
        goal_msg.pose.position.y = goal[1]
        goal_msg.pose.orientation.w = 1.0
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f"Published goal: {goal}")
    
    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for grid_pos in path:
            global_pos = self.grid_to_global(grid_pos)
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = global_pos[0]
            pose.pose.position.y = global_pos[1]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_publisher.publish(path_msg)
        self.get_logger().info("Published path")

def main():
    rclpy.init()
    node = CustomNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
