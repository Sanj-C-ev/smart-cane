import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/goal_position', 10)
        self.timer = self.create_timer(1.0, self.publish_goal)  # Publish every 1 second
        self.get_logger().info("Goal Publisher Node Initialized.")
        self.goal_x = 110.0
        self.goal_y = 110.0
        
    def publish_goal(self):
        """ Publish a goal position to the topic. """
        goal_msg = Float32MultiArray()
        
        # Example goal: (x, y) coordinates
        goal_x = self.goal_x   # Set the goal x position
        goal_y = self.goal_y   # Set the goal y position

        goal_msg.data = [goal_x, goal_y]
        
        self.publisher_.publish(goal_msg)
        self.get_logger().info(f"Goal published: ({goal_x}, {goal_y})")

def main():
    rclpy.init()
    node = GoalPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
