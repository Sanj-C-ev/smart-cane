import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from queue import PriorityQueue

class CustomNavigation(Node):
    def __init__(self):
        super().__init__('custom_navigation')
        
        self.create_subscription(OccupancyGrid, '/occupancy_grid', self.map_callback, 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/robot_goal', 10)
        self.path_publisher = self.create_publisher(Path, '/robot_path', 10)
        self.create_subscription(OccupancyGrid, '/local_costmap', self.costmap_callback, 10)
        self.occupancy_grid = None
        self.resolution = 0.025 # Assuming 5 cm per cell
        self.width = 500  # Grid width
        self.height = 500  # Grid height
        self.origin_x = -5.0  # Map origin (meters)
        self.origin_y = -5.0
        self.costmap = None
        self.global_goal = (15.0, 15.0)  # Global goal outside the known map
        
    def map_callback(self, msg):
        self.occupancy_grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        
        self.try_planning()
        
    def costmap_callback(self, msg):
        self.costmap = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.try_planning()

    def try_planning(self):
        if self.occupancy_grid is None or self.costmap is None:
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
        start = self.global_to_grid((0.0, 0.0))  # Assume robot starts at (0,0)
        path = self.d_lite(start, target_frontier)
        
        if path:
            self.publish_path(path)
            final_goal = self.grid_to_global(path[-1])
            self.publish_goal(final_goal)
    
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