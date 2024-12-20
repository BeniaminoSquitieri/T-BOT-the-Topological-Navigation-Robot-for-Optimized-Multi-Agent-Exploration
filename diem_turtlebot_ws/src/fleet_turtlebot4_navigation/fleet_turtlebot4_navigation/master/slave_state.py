# slave_state.py

import rclpy

class SlaveState:
    """
    Class to manage the state of each slave robot.
    This class is used by the master node to track and manage individual slaves in a fleet.

    Attributes:
        slave_ns (str): The unique namespace or identifier for the slave robot (e.g., "robot_1").
        publisher (rclpy.Publisher): A ROS publisher for sending navigation commands to this slave.
        assigned_waypoints (list): A list of waypoints (graph nodes) assigned to this slave.
        current_waypoint_index (int): Tracks the current waypoint index the slave is navigating to.
        last_seen_time (float): Timestamp (in seconds) of the last communication (e.g., heartbeat) from the slave.
        initial_x (float): Initial X-coordinate of the slave's starting position in the map.
        initial_y (float): Initial Y-coordinate of the slave's starting position in the map.
        initial_orientation (str): Initial orientation of the slave (e.g., "NORTH", "EAST", etc.).
        waiting (bool): Indicates whether the slave is waiting for an available edge to proceed.
        current_node (str): The label of the current graph node (vertex) where the slave is located.
    """
    def __init__(self, slave_ns: str, publisher: rclpy.publisher):
        """
        Initialize a new instance of the SlaveState class.

        Args:
            slave_ns (str): Namespace or unique identifier for the slave robot.
            publisher (rclpy.Publisher): Publisher to send navigation commands to the slave.
        """
        self.slave_ns = slave_ns
        self.publisher = publisher
        self.assigned_waypoints = []
        self.current_waypoint_index = 0
        self.last_seen_time = 0.0
        self.initial_x = None
        self.initial_y = None
        self.initial_orientation = None
        self.waiting = False
        self.current_node = None
