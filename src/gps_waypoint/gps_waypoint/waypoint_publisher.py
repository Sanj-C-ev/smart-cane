#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
import requests
import math

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        self.destination = None
        self.current_position = None
        self.waypoints = []
        self.current_waypoint_index = 0
        
        # Subscribers
        self.dest_sub = self.create_subscription(NavSatFix, 'destination', self.set_destination, 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/sensors/gps', self.update_position, 10)
        
        # Publisher for next waypoint
        self.waypoint_pub = self.create_publisher(Point, 'next_waypoint', 10)
        
        # Timer for continuous updates
        self.timer = self.create_timer(5.0, self.update_waypoints)

    def set_destination(self, msg):
        self.destination = (msg.latitude, msg.longitude)
        self.get_logger().info(f"📍 New destination set: Lat={msg.latitude}, Lon={msg.longitude}")
        self.waypoints = []
        self.current_waypoint_index = 0

    def update_position(self, msg):
        self.current_position = (msg.latitude, msg.longitude)
        self.check_waypoint_reached()

    def check_waypoint_reached(self):
        if not self.waypoints or not self.current_position:
            return
            
        current_lat, current_lon = self.current_position
        waypoint_lat, waypoint_lon = self.waypoints[self.current_waypoint_index]
        
        distance = self.haversine(current_lon, current_lat, waypoint_lon, waypoint_lat)
        
        if distance < 10:  # 10 meters threshold
            if self.current_waypoint_index < len(self.waypoints) - 1:
                self.current_waypoint_index += 1
                self.publish_next_waypoint()
            else:
                self.get_logger().info("🎉 Destination reached!")

    def update_waypoints(self):
        if not self.destination or not self.current_position:
            return
            
        current_lat, current_lon = self.current_position
        dest_lat, dest_lon = self.destination
        
        url = f"http://router.project-osrm.org/route/v1/foot/{current_lon},{current_lat};{dest_lon},{dest_lat}?overview=full&geometries=geojson"
        
        try:
            response = requests.get(url)
            data = response.json()
            
            if data['code'] == 'Ok':
                new_waypoints = [(coord[1], coord[0]) for coord in data['routes'][0]['geometry']['coordinates']]
                
                # Only update if we got new waypoints
                if new_waypoints:
                    self.waypoints = new_waypoints
                    # Reset index if we got a completely new route
                    if self.current_waypoint_index >= len(self.waypoints):
                        self.current_waypoint_index = 0
                    self.publish_next_waypoint()
                    self.get_logger().info("🔄 Updated waypoints based on current position")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to fetch route: {e}")

    def publish_next_waypoint(self):
        if not self.waypoints:
            return
            
        waypoint = self.waypoints[self.current_waypoint_index]
        msg = Point()
        msg.x = waypoint[1]  # Longitude
        msg.y = waypoint[0]  # Latitude
        msg.z = 0.0
        
        self.waypoint_pub.publish(msg)
        remaining = len(self.waypoints) - self.current_waypoint_index - 1
        self.get_logger().info(f"➡️ Next waypoint {self.current_waypoint_index+1}/{len(self.waypoints)}: Lat={waypoint[0]:.6f}, Lon={waypoint[1]:.6f} | {remaining} remaining", throttle_duration_sec=2)

    @staticmethod
    def haversine(lon1, lat1, lon2, lat2):
        """Calculate the great circle distance between two points on the earth"""
        # Convert decimal degrees to radians 
        lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])
        
        # Haversine formula 
        dlon = lon2 - lon1 
        dlat = lat2 - lat1 
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a)) 
        r = 6371000  # Radius of earth in meters
        return c * r

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()