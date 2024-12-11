import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import json
import sys

class WaypointPublisher(Node):
    """
    A ROS 2 node to publish waypoints from a JSON file as RViz markers.
    Each waypoint is visualized as a spherical marker in the RViz visualization tool.
    """

    def __init__(self, json_path):
        """
        Initialize the waypoint publisher node.

        Args:
            json_path (str): Path to the JSON file containing waypoint data.
        """
        # Initialize the ROS 2 node with the name 'waypoint_marker_publisher'
        super().__init__('waypoint_marker_publisher')

        # Publisher for publishing MarkerArray messages on the '/waypoints' topic
        self.publisher_ = self.create_publisher(MarkerArray, '/waypoints', 10)

        # Store the JSON path for loading waypoints
        self.json_path = json_path

        # Create a marker array from the JSON waypoints
        self.marker_array = self.create_marker_array()

        # Timer to publish markers every second
        self.timer = self.create_timer(1.0, self.publish_markers)

    def create_marker_array(self):
        """
        Create a MarkerArray from the JSON waypoints.

        Returns:
            MarkerArray: An array of markers representing waypoints.
        """
        marker_array = MarkerArray()

        # Load waypoints from the JSON file
        with open(self.json_path, 'r') as f:
            data = json.load(f)

        # Iterate through the nodes in the JSON file and create markers
        for idx, node in enumerate(data["nodes"]):
            marker = Marker()

            # Set marker header with frame_id and timestamp
            marker.header.frame_id = "map"  # Ensure the frame is "map" for RViz
            marker.header.stamp = self.get_clock().now().to_msg()

            # Namespace and unique ID for the marker
            marker.ns = "waypoints"
            marker.id = idx

            # Marker type and action
            marker.type = Marker.SPHERE  # Spherical markers for waypoints
            marker.action = Marker.ADD

            # Set position of the marker
            marker.pose.position.x = node["x"]
            marker.pose.position.y = node["y"]
            marker.pose.position.z = 0.0  # Keep markers on the 2D plane

            # No rotation for the marker
            marker.pose.orientation.w = 1.0

            # Set marker size (scale)
            marker.scale.x = 0.5  # Diameter of 0.5 units
            marker.scale.y = 0.5
            marker.scale.z = 0.0  # 2D representation

            # Set marker color (red with full opacity)
            marker.color.a = 1.0  # Fully opaque
            marker.color.r = 1.0  # Red channel
            marker.color.g = 0.0  # Green channel
            marker.color.b = 0.0  # Blue channel

            # Append the marker to the MarkerArray
            marker_array.markers.append(marker)

        return marker_array

    def publish_markers(self):
        """
        Publish the MarkerArray on the '/waypoints' topic.
        """
        self.publisher_.publish(self.marker_array)
        self.get_logger().info("Publishing waypoints as markers")

def main(args=None):
    """
    Main entry point for the waypoint marker publisher.
    The JSON path must be provided as a command-line argument.
    """
    if len(sys.argv) < 2:
        print("Usage: ros2 run <your_package> publish_waypoints_from_json.py <path_to_json>")
        return

    # Path to the JSON file containing waypoints
    json_path = sys.argv[1]

    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create and spin the WaypointPublisher node
    waypoint_publisher = WaypointPublisher(json_path)
    rclpy.spin(waypoint_publisher)

    # Cleanup when the node shuts down
    waypoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
