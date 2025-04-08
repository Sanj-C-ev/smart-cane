#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
import math

class IndoorGoalProcessor(Node):
    def __init__(self):
        super().__init__('indoor_goal_processor')
        
        # Current pose subscriber
        self.current_pose = None
        self.create_subscription(
            PoseStamped,
            '/smart_cane_pose',
            self.pose_callback,
            10
        )
        
        # Goal publishers and subscribers
        self.waypoint_pub = self.create_publisher(
            PoseStamped, 
            '/next_waypoint', 
            10
        )
        
        self.create_subscription(
            Point,
            '/indoor_goal',
            self.goal_callback,
            10
        )
        
        # For debugging/console input
        self.create_subscription(
            String,
            '/goal_input',
            self.string_input_callback,
            10
        )
        
        self.get_logger().info("Indoor Goal Processor ready")

    def pose_callback(self, msg):
        """Update current pose from /smart_cane_pose"""
        self.current_pose = msg.pose
        self.get_logger().debug(f"Updated current pose: {msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}")

    def string_input_callback(self, msg):
        """Handle string input (for debugging)"""
        try:
            x, y = map(float, msg.data.split(','))
            goal = Point()
            goal.x = x
            goal.y = y
            goal.z = 0.0  # Assuming 2D navigation
            self.process_goal(goal)
        except ValueError:
            self.get_logger().error("Invalid input format. Use 'x,y'")

    def goal_callback(self, msg):
        """Process incoming Point goal"""
        self.process_goal(msg)

    def process_goal(self, goal_point):
        """Convert goal point to PoseStamped and publish"""
        if not self.validate_goal(goal_point):
            return
            
        goal_pose = PoseStamped()
        
        # Set header
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = 'map'  # Same frame as /smart_cane_pose
        
        # Set position
        goal_pose.pose.position.x = goal_point.x
        goal_pose.pose.position.y = goal_point.y
        goal_pose.pose.position.z = 0.0  # 2D navigation
        
        # Set orientation (pointing towards goal)
        if self.current_pose:
            dx = goal_point.x - self.current_pose.position.x
            dy = goal_point.y - self.current_pose.position.y
            yaw = math.atan2(dy, dx)
            
            # Convert yaw to quaternion
            goal_pose.pose.orientation.z = math.sin(yaw/2)
            goal_pose.pose.orientation.w = math.cos(yaw/2)
        else:
            # Default orientation if current pose unknown
            goal_pose.pose.orientation.w = 1.0
            self.get_logger().warn("Current pose unknown - using neutral orientation")
        
        self.waypoint_pub.publish(goal_pose)
        self.get_logger().info(
            f"Published new waypoint: ({goal_point.x:.2f}, {goal_point.y:.2f}) "
            f"with orientation: {goal_pose.pose.orientation.z:.2f}"
        )

    def validate_goal(self, goal_point):
        """Check if goal is valid"""
        # Check for NaN/inf values
        if not all(math.isfinite(v) for v in [goal_point.x, goal_point.y]):
            self.get_logger().error("Invalid goal coordinates (NaN/inf)")
            return False
            
        # Check if goal is different from current position
        if self.current_pose:
            dist = math.sqrt(
                (goal_point.x - self.current_pose.position.x)**2 +
                (goal_point.y - self.current_pose.position.y)**2
            )
            if dist < 0.1:  # 10cm threshold
                self.get_logger().warn("Goal too close to current position")
                return False
                
        return True

def main(args=None):
    rclpy.init(args=args)
    node = IndoorGoalProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()