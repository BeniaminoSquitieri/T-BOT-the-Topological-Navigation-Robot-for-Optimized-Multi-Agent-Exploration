import json
from std_msgs.msg import String

from .slave_state import SlaveState
from .path_calculation import calculate_undirected_cpp_route

class MasterCallbacks:
    """
    Callback methods for the Master node.

    Questa classe incapsula le funzionalità essenziali richieste da un nodo master in un
    sistema di robotica distribuita. Gestisce compiti come:
      - Pubblicazione del grafo di navigazione.
      - Calcolo del percorso globale (Closed Path Planning - CPP).
      - Registrazione di nuovi robot slave.
      - Elaborazione degli aggiornamenti di stato della navigazione provenienti dai robot slave.
      - Gestione dei timeout per i robot slave inattivi.

    Separando questi callback in una propria classe, il codice promuove modularità e
    riutilizzabilità, permettendo a diverse implementazioni del nodo master di ereditare e
    utilizzare queste funzionalità di base.
    """

    def __init__(self):
        """
        Inizializza la classe MasterCallbacks.

        Imposta gli attributi iniziali necessari per la gestione del percorso globale CPP.
        Questo percorso è condiviso tra tutti i robot slave per coordinare i loro compiti di navigazione.
        """
        self.global_cpp_route = []  # Conterrà il percorso Euleriano comune per la navigazione.
        self.first_waypoint_phase = True  # Indica se si è nella fase iniziale di assegnazione del primo waypoint.
        self.slaves_reached_first = set()  # Tiene traccia dei robot slave che hanno raggiunto il primo waypoint.

    def publish_navigation_graph(self):
        """
        Pubblica il grafo di navigazione su un topic ROS2.

        Questo metodo serializza l'attuale grafo di navigazione (`self.full_graph`) in formato JSON
        e lo pubblica su un topic ROS2 designato (`self.graph_publisher`). Il grafo include nodi con
        le loro coordinate spaziali e archi con le loro distanze di percorrenza associate.

        L'attributo 'weight' di ogni arco è utilizzato per rappresentare il tempo di percorrenza,
        calcolato come distanza divisa per un fattore di velocità costante (0.31 unità/sec).
        """
        # Inizializza un messaggio di tipo String per il grafo.
        graph_msg = String()

        # Prepara i dati del grafo in un formato serializzabile in JSON.
        graph_data = {
            'nodes': [],
            'edges': []
        }

        # Itera su tutti i nodi nel grafo completo e aggiunge i loro dettagli a graph_data.
        for node, data in self.full_graph.nodes(data=True):
            graph_data['nodes'].append({
                'label': node,
                'x': data.get('x', 0.0),
                'y': data.get('y', 0.0),
                # L'orientamento può essere aggiunto se necessario.
                # 'orientation': data.get('orientation', 0.0)
            })

        # Itera su tutti gli archi nel grafo completo e aggiunge i loro dettagli a graph_data.
        for u, v, data in self.full_graph.edges(data=True):
            if u < v:  # Evita duplicati in un grafo non diretto.
                dist_value = data.get('weight', 1.0)  # Distanza predefinita se non specificata.
                graph_data['edges'].append({
                    'source': u,
                    'target': v,
                    'distance': dist_value
                })

        # Serializza i dati del grafo in JSON e assegna al messaggio.
        graph_msg.data = json.dumps(graph_data)

        # Pubblica il messaggio serializzato del grafo.
        self.graph_publisher.publish(graph_msg)

        # Log dell'azione di pubblicazione a livello DEBUG.
        self.get_logger().debug("Navigation graph published.")

    def compute_global_cpp_route(self):
        """
        Calcola il percorso globale di Closed Path Planning (CPP) sul grafo di navigazione.

        Questo metodo genera una lista di waypoints (`self.global_cpp_route`) che rappresentano un circuito Euleriano
        che attraversa ogni arco esattamente una volta. Utilizza la funzione `calculate_undirected_cpp_route`
        per calcolare questo percorso basato sui nodi e sul grafo di navigazione.

        Se viene trovato un circuito Euleriano valido, logga il percorso calcolato. Altrimenti, logga un errore.
        """
        waypoints = []

        # Estrae le informazioni dei waypoints da ogni nodo nel grafo.
        for node, data in self.full_graph.nodes(data=True):
            waypoint = {
                'label': node,
                'x': data['x'],
                'y': data['y']
            }
            waypoints.append(waypoint)

        # Calcola il percorso CPP utilizzando l'utilità di calcolo del percorso fornita.
        route_nodes = calculate_undirected_cpp_route(waypoints, self.full_graph, self.get_logger())

        # Assegna il percorso calcolato all'attributo della classe.
        self.global_cpp_route = route_nodes

        # Log del risultato del calcolo del percorso.
        if self.global_cpp_route:
            self.get_logger().info(f"Global CPP route computed: {self.global_cpp_route}")
        else:
            self.get_logger().error("Global CPP route is empty or Euler circuit not found.")

    def slave_registration_callback(self, msg):
        """
        Callback che gestisce la registrazione di un nuovo robot slave.

        Quando un nuovo robot slave pubblica il suo namespace sul topic `/slave_registration`,
        questo callback viene invocato per registrare lo slave all'interno del sistema di tracciamento del master.
        Esegue le seguenti azioni:

        1. Estrae il namespace dello slave dal messaggio in ingresso.
        2. Verifica se lo slave è già registrato.
           - Se non è registrato:
             a. Crea un publisher per inviare comandi di navigazione allo slave.
             b. Inizializza un'istanza di `SlaveState` per tracciare lo stato dello slave.
             c. Aggiunge lo slave al dizionario degli slave del master.
             d. Calcola il percorso CPP globale se non è già stato fatto.
             e. Inizia la ripartizione e assegnazione dei waypoints.
           - Se è già registrato:
             a. Aggiorna il tempo dell'ultima vista dello slave per prevenire timeout.

        Parametri:
        - msg (std_msgs.msg.String): Il messaggio in ingresso contenente il namespace dello slave.
        """
        # Estrae il namespace dello slave dal messaggio, rimuovendo eventuali spazi bianchi.
        slave_ns = msg.data.strip()

        # Ottiene l'orario corrente in secondi dal clock di ROS2.
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Verifica se lo slave è già registrato.
        if slave_ns not in self.slaves:
            # Crea un publisher ROS2 per inviare comandi di navigazione a questo slave.
            publisher = self.create_publisher(String, f"/{slave_ns}/navigation_commands", self.qos_profile)

            # Inizializza una nuova istanza di SlaveState per tracciare lo stato e la comunicazione dello slave.
            slave_state = SlaveState(slave_ns, publisher)
            slave_state.last_seen_time = current_time  # Registra il tempo di registrazione.

            # Aggiunge il nuovo slave al dizionario di tracciamento degli slave del master.
            self.slaves[slave_ns] = slave_state

            # Log della registrazione riuscita del nuovo slave.
            self.get_logger().info(f"Registered new slave: {slave_ns}")

            # Se il percorso CPP globale non è ancora stato calcolato, lo calcola ora.
            if not self.global_cpp_route:
                self.compute_global_cpp_route()

            # Inizia la ripartizione e assegnazione dei waypoints ai robot slave.
            self.waypoint_manager.repartition_and_assign_waypoints()

        else:
            # Se lo slave è già registrato, aggiorna il suo tempo dell'ultima vista per prevenire timeout.
            self.slaves[slave_ns].last_seen_time = current_time
            self.get_logger().debug(f"Updated last seen time for slave: {slave_ns}")

    def navigation_status_callback(self, msg):
        """
        Elabora gli aggiornamenti di stato della navigazione ricevuti dai robot slave.

        I robot slave pubblicano il loro stato di navigazione sul topic `/navigation_status`. Questo callback
        gestisce questi aggiornamenti, eseguendo azioni basate sullo stato riportato, come:

        - Contrassegnare gli slave come pronti.
        - Tracciare l'attraversamento degli archi.
        - Gestire errori riportati dagli slave.
        - Assegnare il prossimo waypoint agli slave dopo una navigazione riuscita.

        Parametri:
        - msg (std_msgs.msg.String): Il messaggio in ingresso contenente lo stato di navigazione dello slave in formato JSON.
        """
        try:
            # Analizza il messaggio JSON in ingresso.
            data = json.loads(msg.data)
            slave_ns = data['robot_namespace']
            status = data['status']
            current_wpt = data['current_waypoint']
            error_message = data.get('error_message', '')
            traversed = data.get('traversed_edge', [])

            # Normalizza l'arco attraversato come una tupla ordinata per garantire la consistenza.
            if len(traversed) == 2:
                edge_key = tuple(sorted(traversed))
            else:
                edge_key = None

        except (json.JSONDecodeError, KeyError) as e:
            # Logga un errore se il messaggio è malformato o mancano campi richiesti.
            self.get_logger().error(f"Invalid navigation status message: {e}")
            return

        # Verifica se lo slave è registrato; se no, ignora il messaggio.
        if slave_ns not in self.slaves:
            # Opzionalmente, gestisci gli slave non registrati, come registrazioni automatiche.
            # self.get_logger().warn(f"Received status from unknown slave '{slave_ns}'. Ignoring message.")
            return

        # Recupera l'istanza di SlaveState per lo slave che sta segnalando.
        slave = self.slaves[slave_ns]

        # Gestisce diversi stati segnalati dallo slave.
        if status == "ready":
            # Se lo slave ha ancora un arco occupato, ignora stati 'ready' ridondanti.
            if slave.current_edge is not None:
                self.get_logger().debug(
                    f"[READY] Slave '{slave_ns}' still has current_edge={slave.current_edge}. Ignoring extra 'ready'."
                )
                return

            # Se lo slave non era già pronto, lo contrassegna come pronto.
            if not slave.ready:
                slave.ready = True
                # self.get_logger().info(f"Slave '{slave_ns}' is ready.")

                # Assegna i waypoints dopo aver confermato che lo slave è pronto.
                self.waypoint_manager.repartition_and_assign_waypoints()
            else:
                # Se lo slave era già contrassegnato come pronto, logga a livello debug.
                self.get_logger().debug(f"Slave '{slave_ns}' was already ready.")

        elif status == "reached":
            # Logga l'arrivo riuscito dello slave a un waypoint.
            # self.get_logger().info(
            #     f"Slave '{slave_ns}' reached waypoint '{current_wpt}'"
            # )
            if edge_key:
                # Logga l'arco attraversato.
                # self.get_logger().info(f"Traversed edge: {edge_key}")
                self.get_logger().info(
                    f"Current occupied edges before freeing: {self.occupied_edges}"
                )
                if edge_key in self.occupied_edges:
                    # Rimuove l'arco dall'insieme degli archi occupati.
                    self.occupied_edges.remove(edge_key)
                    # Rimuove la mappatura dello slave occupante per questo arco.
                    occupant = self.edge_occupants.pop(edge_key, None)
                    self.get_logger().info(
                        f"Edge {edge_key} freed by slave '{slave_ns}'. (Previously occupant={occupant})"
                    )
                else:
                    # Avvisa se l'arco non era segnato come occupato.
                    if edge_key[0] != edge_key[1]:
                        self.get_logger().warn(
                            f"Received 'reached' for edge {edge_key} from slave '{slave_ns}' but it was NOT in occupied_edges."
                        )
            else:
                # Avvisa se nessun arco valido è stato attraversato.
                self.get_logger().warn(f"Slave '{slave_ns}' says 'reached' but no valid edge_key.")

            # Aggiorna il nodo corrente dello slave al waypoint raggiunto.
            slave.current_node = current_wpt
            slave.current_edge = None

            # Assegna il prossimo waypoint allo slave.
            self.waypoint_manager.assign_next_waypoint(slave_ns)

        elif status == "error":
            # Logga l'errore riportato dallo slave.
            self.get_logger().error(f"Slave '{slave_ns}' error: {error_message}")
            # Se un arco era in attraversamento, lo libera.
            if edge_key and edge_key in self.occupied_edges:
                self.occupied_edges.remove(edge_key)
                occupant = self.edge_occupants.pop(edge_key, None)
                self.get_logger().info(
                    f"Edge {edge_key} freed by error from '{slave_ns}'. occupant was={occupant}"
                )

            # Rimuove lo slave dal sistema di tracciamento a causa dell'errore.
            del self.slaves[slave_ns]
            self.get_logger().warn(f"Removed slave '{slave_ns}' due to error.")

        elif status == "traversing":
            # Logga che lo slave sta attualmente attraversando verso un waypoint.
            self.get_logger().debug(
                f"Slave '{slave_ns}' is traversing toward '{current_wpt}'."
            )
            # Aggiungi eventuali aggiornamenti di stato o azioni aggiuntive qui.

        else:
            # Avvisa se viene ricevuto uno stato non gestito.
            self.get_logger().warn(f"Unhandled status '{status}' from '{slave_ns}'.")

    def on_first_waypoint_reached(self, msg: String):
        """
        Callback per gestire le notifiche dai robot slave quando raggiungono il loro primo waypoint.

        Questo metodo traccia quali robot slave hanno segnalato di aver raggiunto il loro primo waypoint.
        Una volta che tutti gli slave attivi hanno segnalato, procede con l'assegnazione dei waypoint
        successivi e smette di ascoltare ulteriori notifiche.

        Parametri:
        - msg (std_msgs.msg.String): Il messaggio in ingresso contenente dati in formato JSON con la chiave 'robot_namespace'.
        """
        # Tenta di decodificare il messaggio JSON.
        try:
            data = json.loads(msg.data)
            slave_ns = data['robot_namespace']
        except (json.JSONDecodeError, KeyError):
            self.get_logger().error("Invalid /first_waypoint_reached message (JSON or missing keys).")
            return

        # Se non si sta aspettando i primi waypoint, ignora la notifica.
        if not self.waiting_for_first_waypoints:
            self.get_logger().debug(f"Ignoring notification from {slave_ns}, not waiting anymore.")
            return

        # Verifica se lo slave esiste.
        if slave_ns not in self.slaves:
            self.get_logger().warn(f"Received notification from unknown slave '{slave_ns}'.")
            return

        # Recupera l'istanza di SlaveState per lo slave che sta segnalando.
        slave = self.slaves[slave_ns]

        # Contrassegna lo slave come aver raggiunto il primo waypoint.
        self.slaves_that_reported_first_wp.add(slave_ns)
        self.get_logger().info(f"Slave '{slave_ns}' has reached its first waypoint.")

        # Imposta il flag in SlaveState.
        slave.has_first_waypoint_assigned = True

        # Verifica se tutti gli slave in attesa hanno raggiunto il primo waypoint.
        waiting_slaves = {s.slave_ns for s in self.slaves.values() if not s.has_first_waypoint_assigned}
        if not waiting_slaves:
            self.get_logger().info("All waiting slaves have reached their first waypoint. Starting subsequent assignments.")

            # Imposta il flag per smettere di aspettare i primi waypoint.
            self.waiting_for_first_waypoints = False

            # Disiscriviti dal topic '/first_waypoint_reached' poiché non è più necessario.
            if self.first_wp_reached_subscriber is not None:
                self.destroy_subscription(self.first_wp_reached_subscriber)
                self.first_wp_reached_subscriber = None
                self.get_logger().info("Unsubscribed from '/first_waypoint_reached'.")

            # Procedi con l'assegnazione dei waypoint successivi.
            self.waypoint_manager.repartition_and_assign_waypoints()
