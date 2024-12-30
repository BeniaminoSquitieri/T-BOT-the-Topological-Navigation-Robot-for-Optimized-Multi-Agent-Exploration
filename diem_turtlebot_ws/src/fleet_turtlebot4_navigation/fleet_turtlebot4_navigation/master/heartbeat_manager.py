from std_msgs.msg import String

class HeartbeatManager:
    """
    Manages heartbeat publishing and monitoring for the master node.
    """
    def __init__(self, node, heartbeat_topic: str = '/master_heartbeat', rate: float = 1.0):
        self.node = node
        self.heartbeat_publisher = self.node.create_publisher(String, heartbeat_topic, 1)
        self.heartbeat_timer = self.node.create_timer(1.0 / rate, self.publish_heartbeat)

    def publish_heartbeat(self):
        """
        Publish a heartbeat message to indicate that the master node is active.
        """
        heartbeat_msg = String()
        heartbeat_msg.data = "alive"
        self.heartbeat_publisher.publish(heartbeat_msg)
        self.node.get_logger().debug("Published master heartbeat.")
