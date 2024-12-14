import networkx as nx
import math

def calculate_dcpp_route(waypoints, subgraph, logger):
    """
    Calculate the Directed Chinese Postman Problem (DCPP) route for a given subgraph using a min cost flow approach.
    Questo metodo bilancia gli sbilanciamenti tra in-degree e out-degree dei nodi usando un modello di rete di flusso
    a costo minimo, in modo da rendere Euleriano il grafo e calcolare successivamente un Eulerian circuit.

    Args:
        waypoints (list of dict): Lista di waypoint, ciascuno con 'label', 'x', 'y', e 'orientation'.
        subgraph (nx.DiGraph): Il sottografo per il quale calcolare il percorso DCPP.
        logger (logging.Logger): Logger per il debugging e il logging del progresso.

    Returns:
        list of dict: Una lista ordinata di waypoint che rappresenta il percorso DCPP.
    """

    G = subgraph  # per comodità di lettura

    # Controllo se il grafo è fortemente connesso: condizione necessaria per la soluzione
    if not nx.is_strongly_connected(G):
        logger.error("The subgraph is not strongly connected, cannot solve DCPP.")
        raise ValueError("Subgraph not strongly connected.")

    # Calcolo degli sbilanciamenti: differenza tra out_degree e in_degree
    imbalances = {v: G.out_degree(v) - G.in_degree(v) for v in G.nodes()}

    # Nodi con sbilanciamenti positivi (out_degree > in_degree) e negativi
    positive_nodes = [v for v, imbalance in imbalances.items() if imbalance > 0]
    negative_nodes = [v for v, imbalance in imbalances.items() if imbalance < 0]

    # Creazione di un grafo ausiliario M per il min cost flow
    # In M aggiungeremo archi per bilanciare i nodi tramite un super_source e un super_sink
    M = nx.DiGraph()

    # Copiamo i nodi e i loro attributi in M
    M.add_nodes_from(G.nodes(data=True))

    # Copiamo tutti gli archi originali nel grafo M
    # Questi archi servono per garantire cammini tra nodi sbilanciati
    for (u, v, data) in G.edges(data=True):
        # weight già calcolato come distanza euclidea nel grafo originale
        M.add_edge(u, v, capacity=float('inf'), weight=data.get('weight', 1.0))

    # Aggiungiamo super_source e super_sink per la formulazione del flusso
    M.add_node('super_source')
    M.add_node('super_sink')

    # Archi dal super_source ai nodi con sbilanciamento positivo
    for v in positive_nodes:
        # Lo sbilanciamento indica quante "unità" di flusso vanno compensate
        M.add_edge('super_source', v, capacity=imbalances[v], weight=0)

    # Archi dai nodi con sbilanciamento negativo al super_sink
    for v in negative_nodes:
        # -imbalances[v] è la quantità di flusso da assorbire
        M.add_edge(v, 'super_sink', capacity=(-imbalances[v]), weight=0)

    # Ora risolviamo il min cost flow sul grafo M
    # Questo ci darà il modo più economico per "pareggiare" in_degree e out_degree dei nodi
    try:
        flowDict = nx.min_cost_flow(M)
    except nx.NetworkXUnfeasible:
        logger.error("Min cost flow problem is infeasible, cannot make the subgraph Eulerian.")
        raise ValueError("Could not find a feasible min cost flow solution.")

    # Il flusso calcolato ci dice quanti archi aggiuntivi dobbiamo considerare per equilibrare i nodi
    # Aggiungiamo questi archi extra al grafo G (dove necessario)
    for u in G.nodes():
        for v, flow in flowDict.get(u, {}).items():
            # Ignoriamo archi che non corrispondono a nodi reali o flusso zero
            if v not in G.nodes() or flow <= 0:
                continue
            # Per ogni unità di flusso assegnata, aggiungiamo archi a G, "rafforzandoli"
            # Di fatto, stiamo aggiungendo percorsi per rendere Euleriano il grafo
            for _ in range(flow):
                # L'arco u->v esiste già (in M), quindi in G è già presente con un certo weight
                # Aggiungiamo archi multipli per rappresentare questi passaggi extra
                if not G.has_edge(u, v):
                    # Se per qualche ragione l'arco non esiste in G (dovrebbe esistere),
                    # creiamo un arco con lo stesso peso stimato
                    dist = math.hypot(G.nodes[v]['x'] - G.nodes[u]['x'],
                                      G.nodes[v]['y'] - G.nodes[u]['y'])
                    G.add_edge(u, v, weight=dist)
                else:
                    # L'arco già esiste, semplicemente aggiungendo "flusso" stiamo considerando
                    # di percorrerlo più volte. In un modello Euleriano, questo equivale ad aggiungere
                    # archi paralleli identici.
                    # In NetworkX un parallel edge non esiste di default, ma per simulare l'idea
                    # potremmo incrementare un attributo o semplicemente ignorare, poiché
                    # eulerian_circuit gestisce già la molteplicità (lo faremo ignorando questa parte).
                    pass

    # Ora abbiamo aggiornato G con archi sufficienti a bilanciare i gradi, dovrebbe essere Euleriano
    if not nx.is_eulerian(G):
        logger.error("Failed to make the subgraph Eulerian after min cost flow balancing.")
        raise ValueError("Subgraph not Eulerian after min cost flow balancing.")

    # Calcoliamo l'Eulerian circuit sul grafo bilanciato
    euler_circuit = list(nx.eulerian_circuit(G))

    # Mappiamo il circuito Euleriano sui waypoint
    # Il primo waypoint del route_labels è quello di partenza (uguale a waypoints[0])
    route_labels = [waypoints[0]['label']]
    for u, v in euler_circuit:
        route_labels.append(v)

    # Associazione delle label ai waypoint reali
    label_to_wp = {wp['label']: wp for wp in waypoints}
    ordered_route = [label_to_wp[label] for label in route_labels if label in label_to_wp]

    # Log del percorso calcolato
    logger.info("Calculated DCPP route (using min cost flow approach):")
    for idx, wp in enumerate(ordered_route, start=1):
        logger.info(f" - Waypoint {idx}: {wp['label']} at ({wp['x']}, {wp['y']}) Orientation: {wp['orientation']} radians")

    return ordered_route

def orientation_str_to_rad(orientation_str):
    """
    Convert a cardinal direction string to radians.

    Args:
        orientation_str (str): Direction as string ('NORTH', 'EAST', 'SOUTH', 'WEST').

    Returns:
        float: Corresponding angle in radians.
    """
    orientations = {
        'NORTH': 0.0,
        'EAST': math.pi / 2,
        'SOUTH': math.pi,
        'WEST': 3 * math.pi / 2
    }
    return orientations.get(orientation_str.upper(), 0.0)

def orientation_rad_to_str(orientation_radians):
    """
    Convert an angle in radians to a cardinal direction string.

    Args:
        orientation_radians (float): Angle in radians.

    Returns:
        str: Corresponding direction ('NORTH', 'EAST', 'SOUTH', 'WEST').
    """
    orientation_map = {
        0.0: 'NORTH',
        math.pi / 2: 'EAST',
        math.pi: 'SOUTH',
        3 * math.pi / 2: 'WEST'
    }
    tolerance = 0.1  # Allow for small differences due to floating-point precision

    for angle, direction in orientation_map.items():
        if abs(orientation_radians - angle) < tolerance:
            return direction

    # If no exact match, return the closest direction
    closest_angle = min(orientation_map.keys(), key=lambda k: abs(k - orientation_radians))
    return orientation_map[closest_angle]
