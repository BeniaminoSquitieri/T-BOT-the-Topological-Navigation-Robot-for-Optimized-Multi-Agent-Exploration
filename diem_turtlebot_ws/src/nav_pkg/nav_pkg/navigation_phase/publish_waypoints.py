import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import json
import sys

class WaypointPublisher(Node):
    def __init__(self, json_path):
        super().__init__('waypoint_marker_publisher')
        self.publisher_ = self.create_publisher(MarkerArray, '/waypoints', 10)
        self.json_path = json_path
        self.marker_array = self.create_marker_array()
        self.timer = self.create_timer(1.0, self.publish_markers)  # Pubblica ogni secondo

    def create_marker_array(self):
        marker_array = MarkerArray()
        # Carica i waypoints dal file JSON
        with open(self.json_path, 'r') as f:
            data = json.load(f)

        for idx, node in enumerate(data["nodes"]):
            marker = Marker()
            marker.header.frame_id = "map"  # Assicurati che il frame sia "map" per RViz
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = node["x"]
            marker.pose.position.y = node["y"]
            marker.pose.position.z = 0.0  # Mantieni i marker sul piano 2D
            marker.pose.orientation.w = 1.0  # Nessuna rotazione
            marker.scale.x = 0.5  # Dimensioni del marker
            marker.scale.y = 0.5
            marker.scale.z = 0.0
            marker.color.a = 1.0  # Opacità del marker
            marker.color.r = 1.0  # Rosso a 0
            marker.color.g = 0.0  # Verde a 1
            marker.color.b = 0.0  # Blu a 0

            marker_array.markers.append(marker)

        return marker_array

    def publish_markers(self):
        self.publisher_.publish(self.marker_array)
        self.get_logger().info("Publishing waypoints as markers")

def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: ros2 run <your_package> publish_waypoints_from_json.py <path_to_json>")
        return

    json_path = sys.argv[1]

    rclpy.init(args=args)
    waypoint_publisher = WaypointPublisher(json_path)
    rclpy.spin(waypoint_publisher)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
