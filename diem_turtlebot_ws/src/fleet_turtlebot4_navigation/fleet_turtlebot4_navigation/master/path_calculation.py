import networkx as nx

def calculate_undirected_cpp_route(waypoints, subgraph: nx.MultiGraph, logger):
    """
    Calcola un percorso DCPP (Chinese Postman Problem) su un grafo non diretto utilizzando MultiGraph.
    
    Parametri:
        waypoints (list): Lista di dizionari contenenti informazioni sui nodi (label, x, y, ecc.).
        subgraph (networkx.MultiGraph): Sottografo assegnato allo slave per il calcolo del percorso.
        logger: Logger del master node per la registrazione di informazioni e messaggi di errore.
    
    Ritorna:
        list: Percorso (lista di etichette di nodi) che copre tutti gli archi del subgrafo almeno una volta.
    """
    # Verifica se il sottografo è effettivamente un MultiGraph (richiesto per duplicazione di archi).
    if not isinstance(subgraph, nx.MultiGraph):
        logger.error(f"Il grafo non è un MultiGraph, bensì {type(subgraph)}. Impossibile calcolare DCPP.")
        return []

    # 1. Troviamo i nodi con grado dispari nel subgrafo.
    odd_degree_nodes = [node for node, deg in subgraph.degree() if deg % 2 != 0]
    # logger.info(f"Trovati {len(odd_degree_nodes)} nodi con grado dispari: {odd_degree_nodes}")

    # 2. Se il grafo è già euleriano (nessun nodo dispari).
    if len(odd_degree_nodes) == 0:
        try:
            # Calcolo diretto del circuito euleriano utilizzando la funzione di NetworkX.
            euler_circuit = list(nx.eulerian_circuit(subgraph))
            # logger.info("Il grafo è già euleriano. Calcolato eulerian_circuit.")
        except nx.NetworkXError as e:
            # Gestione dell'errore nel caso in cui non sia possibile calcolare il circuito.
            logger.error(f"Errore nel calcolo dell'eulerian_circuit: {e}")
            return []
    else:
        # 3. Se il grafo non è euleriano, risolviamo il problema aggiungendo archi per renderlo euleriano.

        # Creiamo un grafo completo con nodi dispari.
        odd_complete = nx.Graph()

        # Per ogni coppia di nodi dispari, calcoliamo la distanza minima nel subgrafo.
        for i, u in enumerate(odd_degree_nodes):
            for v in odd_degree_nodes[i+1:]:
                try:
                    # Calcola la lunghezza del cammino più corto tra u e v usando i pesi degli archi.
                    dist = nx.shortest_path_length(subgraph, source=u, target=v, weight='weight')
                    # Aggiungi un arco al grafo completo tra i nodi dispari con la distanza calcolata come peso.
                    odd_complete.add_edge(u, v, weight=dist)
                except nx.NetworkXNoPath:
                    # Logga se non esiste un cammino tra due nodi dispari.
                    logger.error(f"Nessun cammino tra {u} e {v} nel subgrafo.")
                    continue

        # Se non ci sono archi nel grafo completo dei nodi dispari, non possiamo procedere.
        if odd_complete.number_of_edges() == 0:
            logger.error("Impossibile calcolare il matching tra i nodi dispari.")
            return []

        # Calcolo del matching di peso minimo utilizzando la funzione min_weight_matching.
        matching = nx.min_weight_matching(odd_complete,  weight='weight')
        # logger.info(f"Matching minimo trovato tra i nodi dispari: {matching}")

        # Aggiungiamo gli archi del matching al subgrafo.
        for (u, v) in matching:
            if subgraph.has_edge(u, v):
                # Se l'arco esiste già, utilizziamo il peso originale del subgrafo.
                original_weight = list(subgraph.get_edge_data(u, v).values())[0]['weight']
            else:
                # Se l'arco non esiste, calcoliamo il peso come la distanza più corta.
                original_weight = nx.shortest_path_length(subgraph, source=u, target=v, weight='weight')
            # Aggiungiamo l'arco con il peso originale al subgrafo.
            subgraph.add_edge(u, v, weight=original_weight)

        # Ora il subgrafo è euleriano, quindi possiamo calcolare il circuito euleriano.
        try:
            euler_circuit = list(nx.eulerian_circuit(subgraph))
            # logger.info("Percorso Euleriano calcolato dopo aver aggiunto gli archi di matching.")
        except nx.NetworkXError as e:
            # Logga l'errore se non è possibile calcolare il circuito.
            logger.error(f"Errore nel calcolo dell'eulerian_circuit: {e}")
            return []

    # 4. Costruiamo la lista ordinata di nodi dal circuito euleriano.
    ordered_route = []
    for u, v in euler_circuit:
        if not ordered_route:
            # Aggiungiamo il primo nodo al percorso.
            ordered_route.append(u)
        # Aggiungiamo il nodo finale dell'arco.
        ordered_route.append(v)

    # Ritorniamo il percorso ordinato che copre tutti gli archi del grafo.
    return ordered_route
