import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class CmdVelMapper(Node):
    def __init__(self):
        super().__init__('cmd_vel_mapper')
        self.subscription = self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_callback,10)
        self.publisher = self.create_publisher(Float32,'/kinesthetic_cmd',10)
        
    def cmd_vel_callback(self,msg):
        angular_z = msg.angular.z
        k_map = self.map_value(angular_z,-1.0,1.0,0.0,180.0)
        
        angle_msg = Float32()
        angle_msg.data = k_map
        self.publisher.publish(angle_msg)

        self.get_logger().info(f"Z: {angular_z} ... K_val: {angle_msg.data}")
    
    def map_value(self,value,fmin,fmax,tmin,tmax):
        return tmin + (value-fmin)*(tmax-tmin)/(fmax-fmin)
    
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()