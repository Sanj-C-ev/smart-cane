#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import csv
import os

class DestinationInputNode(Node):
    def __init__(self):
        super().__init__('destination_input_node')
        self.publisher_ = self.create_publisher(NavSatFix, 'destination', 10)
        
        # Load locations from CSV
        self.locations = self.load_locations()
        
        # Create a timer for manual input
        self.timer = self.create_timer(1.0, self.process_manual_input)

    def load_locations(self):
        locations = {}
        csv_path = os.path.join(os.path.dirname(__file__), 'home/sanj_19/Downloads/nav_way21/nav_way2/amrita_locations.csv')
        try:
            with open(csv_path, mode='r') as csv_file:
                reader = csv.DictReader(csv_file)
                for row in reader:
                    locations[row['name'].lower()] = (float(row['latitude']), float(row['longitude']))
            self.get_logger().info("✅ Locations loaded successfully")
            return locations
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load locations: {e}")
            return {}

    def process_manual_input(self):
        if not self.locations:
            self.get_logger().error("No locations loaded. Cannot process input.")
            return
            
        print("\nAvailable destinations:")
        for location in self.locations.keys():
            print(f"- {location.title()}")
            
        user_input = input("\nEnter your destination: ").strip().lower()
        
        if user_input in self.locations:
            lat, lon = self.locations[user_input]
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'destination'
            msg.latitude = lat
            msg.longitude = lon
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            
            self.publisher_.publish(msg)
            self.get_logger().info(f"✅ Destination set to: {user_input.title()} (Lat={lat}, Lon={lon})")
        else:
            self.get_logger().warn(f"⚠️ Unknown destination: {user_input}")

def main(args=None):
    rclpy.init(args=args)
    node = DestinationInputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()