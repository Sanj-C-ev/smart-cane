import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from queue import PriorityQueue
import os
import datetime
import random


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
        self.path_publisher_dl = self.create_publisher(Path, '/smart_cane_path_dlite', 10)
        self.path_publisher_astar = self.create_publisher(Path, '/smart_cane_path_astar', 10)
        self.path_publisher_rrt = self.create_publisher(Path, '/smart_cane_path_rrt', 10)
    
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
        self.final_path = None


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

        frontiers = self.detect_frontiers()
        if not frontiers:
            self.get_logger().info("No frontiers detected.")
            return

        target = self.select_closest_frontier(frontiers)
        self.plan_path(target)

    def detect_frontiers(self):
        frontiers = []
        for y in range(self.height):
            for x in range(self.width):
                if self.occupancy_grid[y, x] == 0:  # Free cell
                    if self.is_frontier(x, y):
                        frontiers.append((x, y))
        return frontiers
    
    def is_frontier(self, x, y):
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    if self.occupancy_grid[ny, nx] > 50:  # Unknown or obstacle
                        return True
        return False
    
    def select_closest_frontier(self, frontiers):
        goal_x, goal_y = self.global_to_grid(self.global_goal)
        return min(frontiers, key=lambda f: np.hypot(f[0] - goal_x, f[1] - goal_y))
    
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
    
    def plan_path(self, target_frontier):
        """Plans path from current pose to target frontier using different algorithms."""
        start_x = self.current_pose.pose.position.x
        start_y = self.current_pose.pose.position.y
        start = self.global_to_grid((start_x, start_y))  # Dynamic start position
        
        path_dl = self.d_lite(start, target_frontier)
        path_astar = self.a_star(start, target_frontier)
        path_rrt = self.rrt(start, target_frontier)
        
        if path_dl:
            self.publish_path(path_dl, "dl")
        if path_astar:
            self.publish_path(path_astar, "astar")
        if path_rrt:
            self.publish_path(path_rrt, "rrt")
        
        # Optionally save or evaluate paths
        self.save_path(path_dl, "dl")
        self.save_path(path_astar, "astar")
        self.save_path(path_rrt, "rrt")
    
    def d_lite(self, start, goal):
        """Implements the D* Lite algorithm."""
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
                    # Get occupancy and inflation cost
                    occ = self.occupancy_grid[ny, nx]
                    inflation = self.costmap[ny, nx]

                    # Skip if occupied (hard obstacle)
                    if occ >= 50:
                        continue

                    # Optional: discourage moving into unknown space
                    if occ == -1:
                        continue  # or use: inflation += 50

                    # Normalize inflation to a cost factor
                    inflation_cost = inflation / 100.0

                    move_cost = 1 + inflation_cost  # Base cost + inflation penalty
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


    def a_star(self, start, goal):
        """Implements the A* algorithm."""
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
                    
                    if occ >= 50:  # Skip occupied cells
                        continue
                    
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

    def rrt(self, start, goal, max_iter=1000, step_size=5, goal_sample_rate=0.1):
        """Improved RRT implementation with path reconstruction."""
        tree = {start: None}  # Stores nodes and their parents
        
        for _ in range(max_iter):
            # Sample with some probability towards the goal
            if random.random() < goal_sample_rate:
                sample = goal
            else:
                sample = (random.randint(0, self.width-1), 
                        random.randint(0, self.height-1))
            
            # Find nearest node
            nearest = min(tree.keys(), 
                        key=lambda p: np.hypot(p[0]-sample[0], p[1]-sample[1]))
            
            # Move towards sample
            direction = np.array([sample[0]-nearest[0], sample[1]-nearest[1]])
            dist = np.hypot(direction[0], direction[1])
            if dist == 0:
                continue
                
            direction = direction / dist  # Normalize
            new_point = (int(nearest[0] + direction[0] * step_size),
                        int(nearest[1] + direction[1] * step_size))
            
            # Check bounds and collisions
            if (0 <= new_point[0] < self.width and 
                0 <= new_point[1] < self.height and
                not self.check_collision(nearest, new_point)):
                
                tree[new_point] = nearest  # Record parent
                
                # Check if reached goal
                if np.hypot(new_point[0]-goal[0], new_point[1]-goal[1]) < step_size:
                    return self.reconstruct_path(tree, new_point)
        
        return None  # No path found

    def check_collision(self, p1, p2):
        """Check if path between p1 and p2 collides with obstacles."""
        # Bresenham's line algorithm to check all cells along the path
        x0, y0 = p1
        x1, y1 = p2
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        
        while True:
            # Check current cell
            if (0 <= x0 < self.width and 0 <= y0 < self.height):
                if self.occupancy_grid[y0, x0] >= 50:  # Occupied
                    return True
            else:  # Out of bounds
                return True
                
            if x0 == x1 and y0 == y1:
                break
                
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
        
        return False

    def reconstruct_path(self, tree, node):
        """Reconstruct path from start to given node."""
        path = []
        while node is not None:
            path.append(node)
            node = tree[node]
        path.reverse()
        return path

    
    def publish_path(self, path, path_type):
        """Publishes the path to a specific topic."""
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
        
        if path_type == "dl":
            self.path_publisher_dl.publish(path_msg)
        elif path_type == "astar":
            self.path_publisher_astar.publish(path_msg)
        elif path_type == "rrt":
            self.path_publisher_rrt.publish(path_msg)
        
        self.get_logger().info(f"Published {path_type} path")
    
    def save_path(self, path, path_type):
        """Save path to file."""
        if not path:
            self.get_logger().warn(f"No {path_type} path to save.")
            return
        
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_dir = os.path.expanduser("~/navigation_paths")
        os.makedirs(save_dir, exist_ok=True)
        filename = os.path.join(save_dir, f"{path_type}_path_{timestamp}.csv")

        with open(filename, 'w') as f:
            f.write("x,y\n")
            for grid_pos in path:
                gx, gy = self.grid_to_global(grid_pos)
                f.write(f"{gx},{gy}\n")

        self.get_logger().info(f"{path_type.capitalize()} path saved to: {filename}")


def main():
    rclpy.init()
    node = CustomNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
        node.save_path(node.final_path, "dl")
        node.save_path(node.final_path, "astar")
        node.save_path(node.final_path, "rrt")
    finally:
        node.destroy_node()
        rclpy.shutdown()
