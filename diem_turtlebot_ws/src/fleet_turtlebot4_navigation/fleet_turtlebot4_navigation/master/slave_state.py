class SlaveState:
    def __init__(self, slave_ns, publisher):
        self.slave_ns = slave_ns
        self.publisher = publisher
        self.current_node = None
        self.assigned_waypoints = []
        self.current_waypoint_index = 0
        self.current_edge = None
        self.waiting = False
        self.ready = False
        self.last_seen_time = 0.0

        self.has_first_waypoint_assigned = False
