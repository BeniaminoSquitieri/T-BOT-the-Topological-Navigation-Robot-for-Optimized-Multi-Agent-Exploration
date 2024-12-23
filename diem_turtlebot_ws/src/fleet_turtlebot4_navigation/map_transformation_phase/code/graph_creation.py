# graph_creation.py

import networkx as ntx
from sklearn.cluster import DBSCAN
from scipy.spatial import cKDTree
from skimage.draw import line as draw_line
import numpy as np
import logging
from scipy.ndimage import convolve

def check_line_passes_through_skeleton(node1, node2, skeleton, tolerance):
    """
    Verifica se una linea tra due nodi passa attraverso la skeleton con una certa tolleranza.
    
    Parametri:
        node1 (tuple): Coordinate del primo nodo (y, x).
        node2 (tuple): Coordinate del secondo nodo (y, x).
        skeleton (numpy.ndarray): La mappa scheletrica di Voronoi.
        tolerance (float): Proporzione minima di pixel lungo la linea che devono appartenere alla skeleton.
                           Valore tra 0 e 1. Default è 0.06 (6%).
    
    Ritorna:
        bool: True se la linea passa attraverso la skeleton con la tolleranza richiesta, False altrimenti.
    """
    y0, x0 = node1
    y1, x1 = node2

    # Ottieni i pixel lungo la linea usando l'algoritmo di Bresenham
    rr, cc = draw_line(y0, x0, y1, x1)

    # Verifica che gli indici siano all'interno dei limiti della skeleton
    valid_indices = (rr >= 0) & (rr < skeleton.shape[0]) & \
                    (cc >= 0) & (cc < skeleton.shape[1])
    
    rr = rr[valid_indices]
    cc = cc[valid_indices]
    # print(cc)
    if rr.size == 0 or cc.size == 0:
        logging.debug(f"Line from {node1} to {node2} is out of skeleton bounds.")
        return False

    # Controlla quanti pixel lungo la linea sono nella skeleton
    line_skeleton = skeleton[rr, cc]
    # print(line_skeleton)
    proportion = np.sum(line_skeleton == 1) / len(line_skeleton)
    # print(proportion)
    if proportion >= tolerance:
        logging.debug(f"Line from {node1} to {node2} passes through the skeleton with proportion {proportion:.2f}.")
        return True
    else:
        logging.debug(f"Line from {node1} to {node2} fails proportion {proportion:.2f}.")
        return False


def create_topological_graph_using_skeleton(voronoi_skeleton, config):
    """
    Crea un grafo topologico basato sulla skeleton di Voronoi.
    
    Parametri:
        voronoi_skeleton (numpy.ndarray): La mappa scheletrica di Voronoi.
        config (Config): Oggetto di configurazione con parametri dinamici.
    
    Ritorna:
        networkx.Graph: Il grafo topologico non orientato.
    """
    # Cambia il grafo in non orientato
    topo_map = ntx.Graph()

    # Definisci un kernel per contare i vicini (escludendo il pixel centrale)
    kernel = np.array([[1, 1, 1],
                       [1, 0, 1],
                       [1, 1, 1]], dtype=int)

    # Conta il numero di vicini per ogni pixel nella skeleton
    neighbor_count = convolve(voronoi_skeleton.astype(int), kernel, mode='constant', cval=0)

    # Identifica i nodi agli incroci o agli endpoint (dove il numero di vicini è diverso da 2)
    node_positions = np.column_stack(np.where((voronoi_skeleton == 1) & (neighbor_count != 2)))

    if node_positions.size == 0:
        logging.error("Nessun nodo rilevato nella skeleton. Controlla il processo di pulizia della mappa.")
        raise ValueError("Nessun nodo rilevato nella skeleton.")

    # Applica DBSCAN per unire nodi vicini
    clustering = DBSCAN(eps=config.merge_threshold, min_samples=1).fit(node_positions)

    # Calcola il centroide di ogni cluster per ottenere nodi fusi e assegnare etichette
    fused_nodes = []
    nodes_with_labels = []
    for i, cluster_id in enumerate(np.unique(clustering.labels_), start=1):
        cluster_points = node_positions[clustering.labels_ == cluster_id]
        centroid = np.mean(cluster_points, axis=0).astype(int)
        fused_nodes.append(tuple(centroid))
        label = f"node_{i}"
        nodes_with_labels.append((tuple(centroid), {"label": label}))

    logging.info(f"Fusione dei nodi completata. Numero di nodi dopo la fusione: {len(fused_nodes)}")

    # Aggiungi i nodi fusi al grafo con le etichette
    topo_map.add_nodes_from(nodes_with_labels)

    # Costruisci un KD-Tree per cercare efficientemente nodi vicini
    node_tree = cKDTree(fused_nodes)

    # Trova coppie di nodi entro la distanza massima
    pairs = node_tree.query_pairs(r=config.max_connection_distance)

    logging.info(f"Numero di coppie di nodi entro {config.max_connection_distance} pixel: {len(pairs)}")

    if not pairs:
        logging.warning("Nessuna coppia di nodi trovata entro la distanza massima di connessione.")

    return topo_map
