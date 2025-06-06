from std_msgs.msg import String

class HeartbeatManager:
    """
    Manages heartbeat publishing and monitoring for the master node.
    
    In a distributed system with multiple robots (masters and slaves), it is crucial to
    ensure that there is always an active master to coordinate tasks. The HeartbeatManager
    plays a vital role in this by:
      - Continuously publishing heartbeat messages to signal that the master node is active.
      - Optionally, it can be extended to monitor heartbeats from other nodes or implement
        additional functionalities related to master node health.
    
    This class is intended to be instantiated by a master node, which requires sending
    periodic heartbeat messages to inform other nodes (slaves) of its active status.
    """
    
    def __init__(self, node, qos_profile, heartbeat_topic: str = '/master_heartbeat'):
        """
        Initializes the HeartbeatManager.
        
        Parameters:
        - node (rclpy.node.Node): The ROS2 node instance that will utilize this HeartbeatManager.
        - qos_profile (QoSProfile): The QoSProfile to be used for publishing heartbeat messages.
        - heartbeat_topic (str): The ROS2 topic name where heartbeat messages will be published.
          Defaults to '/master_heartbeat'.
        - rate (float): The frequency (in Hz) at which heartbeat messages are published.
          Defaults to 10.0 Hz.
        """
        self.node = node  # Reference to the ROS2 node
        self.heartbeat_topic = heartbeat_topic
        self.qos_profile = qos_profile  # Use the shared QoSProfile
        self.heartbeat_publisher = self.node.create_publisher(String, self.heartbeat_topic, self.qos_profile)
        self.heartbeat_timer = None  # Timer will be initialized in start_publishing()
        
    def start_publishing(self):
        """
        Starts the heartbeat publishing by creating a timer.
        
        This method should be called when the node assumes the Master role.
        """
        if self.heartbeat_timer is None:
            self.heartbeat_timer = self.node.create_timer(1.0 , self.publish_heartbeat)
        else:
            self.node.get_logger().warn("[HeartbeatManager] Heartbeat publishing is already active.")
    
    def stop_publishing(self):
        """
        Stops the heartbeat publishing by cancelling the timer.
        """
        if self.heartbeat_timer is not None:
            self.heartbeat_timer.cancel()
            self.heartbeat_timer = None
            self.node.get_logger().info("[HeartbeatManager] Stopped publishing heartbeats.")
    
    def publish_heartbeat(self):
        """
        Publishes a heartbeat message to indicate that the master node is active.
        
        This method is periodically called by the ROS2 timer set during start_publishing().
        """
        heartbeat_msg = String()
        heartbeat_msg.data = "alive"
        self.heartbeat_publisher.publish(heartbeat_msg)
        self.node.get_logger().debug("Published master heartbeat.")
