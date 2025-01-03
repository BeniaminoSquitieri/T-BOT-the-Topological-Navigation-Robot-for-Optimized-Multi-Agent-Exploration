class SlaveState:
    """
    Represents the state and communication details of a slave robot in the fleet.
    
    The `SlaveState` class encapsulates all relevant information about a slave robot, including
    its namespace, current navigation status, assigned waypoints, and communication publishers.
    This class is essential for the master node to effectively manage and coordinate multiple
    slaves, ensuring efficient navigation and task execution across the robot fleet.
    
    Attributes:
        slave_ns (str): 
            The unique namespace identifier for the slave robot (e.g., 'robot1', 'robot2').
            This namespace is used to route messages specifically to and from the slave.
        
        publisher (rclpy.publisher.Publisher): 
            A ROS2 publisher instance dedicated to sending navigation commands to this slave.
            It publishes messages of type `std_msgs/String` on the topic `/{slave_ns}/navigation_commands`.
        
        current_node (str or None): 
            The label of the current node (location) where the slave robot is positioned.
            This attribute is updated as the slave navigates through different waypoints.
        
        assigned_waypoints (list of dict): 
            A list of waypoints assigned to the slave for navigation. Each waypoint is a 
            dictionary containing information such as 'label', 'x', 'y', etc. Example:
                [
                    {'label': 'node_1', 'x': 1.0, 'y': 2.0},
                    {'label': 'node_2', 'x': 3.0, 'y': 4.0},
                    # ... more waypoints ...
                ]
        
        current_waypoint_index (int): 
            The index of the next waypoint in `assigned_waypoints` that the slave is scheduled to navigate to.
            This index increments as the slave progresses through its assigned waypoints.
        
        current_edge (tuple or None): 
            A tuple representing the current edge (path between two nodes) that the slave is traversing.
            For example, ('node_1', 'node_2'). This attribute helps in tracking edge occupancy and avoiding
            task conflicts with other slaves.
        
        waiting (bool): 
            A flag indicating whether the slave is currently waiting for a new waypoint assignment.
            When `True`, the slave is idle and ready to receive new navigation commands.
        
        ready (bool): 
            A flag indicating whether the slave has successfully received and is ready to execute assigned waypoints.
            This flag is set to `True` after the slave acknowledges its readiness to the master node.
        
        last_seen_time (float): 
            The timestamp (in seconds) of the last heartbeat message received from the slave.
            This is used to monitor the slave's activity and detect any potential timeouts.
        
        has_first_waypoint_assigned (bool): 
            A flag indicating whether the first waypoint has been assigned to the slave.
            This is particularly useful during the initial assignment phase to trigger any necessary setup 
            or synchronization actions once the first waypoint is in place.
    """
    
    def __init__(self, slave_ns, publisher):
        """
        Initializes a new instance of the SlaveState class.
        
        Parameters:
            slave_ns (str): 
                The unique namespace identifier for the slave robot. This namespace is used to 
                direct navigation commands and receive status updates specific to this slave.
                Example: 'robot1', 'robot2', etc.
                
            publisher (rclpy.publisher.Publisher): 
                A ROS2 publisher instance dedicated to sending navigation commands to this slave.
                It publishes messages of type `std_msgs/String` on the topic `/{slave_ns}/navigation_commands`.
        
        Initializes all attributes to their default states to prepare for tracking and managing the slave's
        navigation tasks and communication.
        """
        # ---------------------------
        # Slave Identification
        # ---------------------------
        self.slave_ns = slave_ns
        """
        The unique namespace identifier for the slave robot.
        
        Example:
            'robot1', 'robot2', etc.
        """
        
        # ---------------------------
        # Communication Publisher
        # ---------------------------
        self.publisher = publisher
        """
        ROS2 publisher for sending navigation commands to the slave.
        
        This publisher sends messages of type `std_msgs/String` to the topic:
            `/{slave_ns}/navigation_commands`
        Example:
            '/robot1/navigation_commands'
        """
        
        # ---------------------------
        # Current Navigation Status
        # ---------------------------
        self.current_node = None
        """
        The label of the current node (location) where the slave robot is positioned.
        
        This attribute is updated as the slave navigates through different waypoints.
        """
        
        self.assigned_waypoints = []
        """
        A list of waypoints assigned to the slave for navigation.
        
        Each waypoint is a dictionary containing information such as 'label', 'x', 'y', etc.
        Example:
            [
                {'label': 'node_1', 'x': 1.0, 'y': 2.0},
                {'label': 'node_2', 'x': 3.0, 'y': 4.0},
                # ... more waypoints ...
            ]
        """
        
        self.current_waypoint_index = 0
        """
        The index of the next waypoint in `assigned_waypoints` that the slave is scheduled to navigate to.
        
        This index increments as the slave progresses through its assigned waypoints.
        """
        
        self.current_edge = None
        """
        A tuple representing the current edge (path between two nodes) that the slave is traversing.
        
        Example:
            ('node_1', 'node_2')
        
        This attribute helps in tracking edge occupancy and avoiding task conflicts with other slaves.
        """
        
        # ---------------------------
        # Status Flags
        # ---------------------------
        self.waiting = False
        """
        Indicates whether the slave is currently waiting for a new waypoint assignment.
        
        When `True`, the slave is idle and ready to receive new navigation commands.
        """
        
        self.ready = False
        """
        Indicates whether the slave has successfully received and is ready to execute assigned waypoints.
        
        This flag is set to `True` after the slave acknowledges its readiness to the master node.
        """
        
        # ---------------------------
        # Heartbeat Monitoring
        # ---------------------------
        self.last_seen_time = 0.0
        """
        The timestamp (in seconds) of the last heartbeat message received from the slave.
        
        This is used to monitor the slave's activity and detect any potential timeouts.
        """
        
        # ---------------------------
        # Waypoint Assignment Tracking
        # ---------------------------
        self.has_first_waypoint_assigned = False
        """
        Indicates whether the first waypoint has been assigned to the slave.
        
        This is particularly useful during the initial assignment phase to trigger any necessary setup 
        or synchronization actions once the first waypoint is in place.
        """
