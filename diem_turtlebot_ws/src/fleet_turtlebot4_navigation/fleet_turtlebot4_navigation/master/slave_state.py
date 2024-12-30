# slave_state.py

class SlaveState:
    """
    Classe per tracciare lo stato di uno slave.
    """
    def __init__(self, slave_ns, publisher):
        self.slave_ns = slave_ns  # Namespace dello slave
        self.publisher = publisher  # Publisher per inviare waypoints
        self.current_node = None  # Nodo corrente dello slave
        self.assigned_waypoints = []  # Lista di waypoints assegnati
        self.current_waypoint_index = 0  # Indice del waypoint corrente
        self.current_edge = None  # Arco attualmente assegnato
        self.waiting = False  # Flag se lo slave è in attesa
        self.ready = False  # Flag se lo slave ha segnalato di essere pronto
        self.last_seen_time = 0.0  # Timestamp dell'ultimo aggiornamento dello slave
