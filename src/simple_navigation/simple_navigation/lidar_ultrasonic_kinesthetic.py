#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class AvoidanceNode(Node):
    def __init__(self):
        super().__init__('avoidance_node')
        
        # Sensor subscription
        self.create_subscription(
            Float32MultiArray, 
            '/sensors/ultrasonic_cluster_1', 
            self.sensor_callback, 
            10
        )
        
        # cmd_vel publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Parameters
        self.safe_distance = 1.5  # meters
        self.close_distance = 0.5  # meters
        self.max_speed = 1.0
        self.min_speed = 0.1
        self.turn_speed = 0.3
        self.angular_speed = 0.5  # rad/s for turns
        
        # State
        self.distances = [4.0, 4.0, 4.0]  # [left, middle, right]
        self.recovery_mode = False
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)
    
    def sensor_callback(self, msg):
        if len(msg.data) == 3:
            self.distances = msg.data
    
    def calculate_velocities(self):
        left, front, right = self.distances
        
        # Calculate linear velocity (0-1.0)
        if front >= self.safe_distance:
            linear = self.max_speed
        elif front <= self.close_distance:
            linear = self.min_speed
        else:
            linear = self.min_speed + (front - self.close_distance) / \
                (self.safe_distance - self.close_distance) * \
                (self.max_speed - self.min_speed)
        
        # Calculate angular velocity (-1.0 to 1.0)
        angular = 0.0  # Default to no turn
        
        # Check if we're stuck (all sides blocked)
        if (front < self.close_distance and 
            left < self.close_distance and 
            right < self.close_distance):
            self.recovery_mode = True
            linear = self.min_speed  # Force minimum speed
            angular = self.angular_speed  # Turn left by default
        
        # Normal obstacle avoidance
        elif front < self.safe_distance:
            self.recovery_mode = False
            
            # Prefer the clearer side
            if left > right and left > self.close_distance:
                angular = -self.angular_speed  # Turn left (positive z)
            elif right > left and right > self.close_distance:
                angular = self.angular_speed  # Turn right (negative z)
            elif left > self.close_distance:
                angular = -self.angular_speed * 0.5  # Slow left turn
            elif right > self.close_distance:
                angular = self.angular_speed * 0.5  # Slow right turn
            else:
                linear = 0.0  # Stop if no clear path
        
        # Reduce speed during turns (except in recovery mode)
        if angular != 0.0 and not self.recovery_mode:
            linear = min(linear, self.turn_speed)
        
        return linear, angular
    
    def publish_cmd_vel(self):
        linear, angular = self.calculate_velocities()
        
        cmd_vel = Twist()
        cmd_vel.linear.x = linear
        cmd_vel.angular.z = angular
        
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Debug output
        self.get_logger().info(
            f"CmdVel: Linear={linear:.2f}, Angular={angular:.2f} | "
            f"Sensors: L={self.distances[0]:.2f}, F={self.distances[1]:.2f}, R={self.distances[2]:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
