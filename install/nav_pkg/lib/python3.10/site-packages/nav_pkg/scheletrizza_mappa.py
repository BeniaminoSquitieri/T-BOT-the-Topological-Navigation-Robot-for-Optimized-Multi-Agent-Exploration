import numpy as np
import cv2
import matplotlib.pyplot as plt
import networkx as nx
from scipy.ndimage import distance_transform_edt
from skimage.morphology import skeletonize
from scipy.spatial import distance

# --- Passo 1: Carica la mappa scheletrizzata ---
def load_map(image_path):
    skeleton = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if skeleton is None:
        raise FileNotFoundError(f"Errore: impossibile aprire il file {image_path}")
    return skeleton

# --- Passo 2: Generazione della Mappa delle Distanze Euclidea ---
def generate_distance_map(binary_map):
    distance_map = distance_transform_edt(1 - binary_map)
    return distance_map

# --- Passo 3: Creazione del Diagramma di Voronoi (Skeletonization) ---
def generate_voronoi_diagram(distance_map):
    skeleton = skeletonize(distance_map > 0)
    return skeleton

# --- Passo 4: Creazione del grafo topologico ---
def create_topological_graph(voronoi_skeleton):
    rows, cols = voronoi_skeleton.shape
    topo_map = nx.Graph()
    
    for x in range(1, rows - 1):
        for y in range(1, cols - 1):
            if voronoi_skeleton[x, y] == 1:
                # Trova i vicini
                neighbors = voronoi_skeleton[x-1:x+2, y-1:y+2].flatten()
                num_neighbors = np.sum(neighbors) - 1  # Rimuovo se stesso
                
                # Identifica nodi (intersezioni e punti finali)
                if num_neighbors > 2:
                    topo_map.add_node((x, y), type='intersection')
                elif num_neighbors == 1:
                    topo_map.add_node((x, y), type='endpoint')

    return topo_map

# --- Passo 5: Aggiunta degli archi ---
def add_edges(topo_map, binary_map):
    visited = set()
    nodes_list = list(topo_map.nodes)

    for (x, y) in nodes_list:
        if (x, y) not in visited:
            current = (x, y)
            path = [current]
            while True:
                visited.add(current)
                neighbors = [(current[0] + dx, current[1] + dy) for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]]
                neighbors = [n for n in neighbors if binary_map[n[0], n[1]] == 1 and n not in visited]

                if len(neighbors) == 1:
                    next_node = neighbors[0]
                    path.append(next_node)
                    current = next_node
                else:
                    break

            if len(path) > 1:
                topo_map.add_edge(path[0], path[-1], weight=len(path))
    
    return topo_map

# --- Passo 6: Accorpamento dei nodi ---
def merge_close_nodes(topo_map, threshold=15):
    nodes_to_merge = list(topo_map.nodes())
    nodes_to_remove = set()

    for i, node1 in enumerate(nodes_to_merge):
        for j in range(i + 1, len(nodes_to_merge)):
            node2 = nodes_to_merge[j]
            dist = distance.euclidean(node1, node2)
            if dist < threshold:
                for neighbor in list(topo_map.neighbors(node2)):
                    topo_map.add_edge(node1, neighbor, weight=topo_map[node2][neighbor]['weight'])
                nodes_to_remove.add(node2)
    
    topo_map.remove_nodes_from(nodes_to_remove)
    return topo_map

# --- Passo 7: Visualizzazione della mappa topologica ---
def visualize_topological_map(skeleton, topo_map):
    plt.figure(figsize=(10, 10))
    plt.imshow(skeleton, cmap=plt.cm.gray)
    
    intersection_nodes = [node for node, data in topo_map.nodes(data=True) if data.get('type') == 'intersection']
    endpoint_nodes = [node for node, data in topo_map.nodes(data=True) if data.get('type') == 'endpoint']
    
    pos = {node: (node[1], node[0]) for node in topo_map.nodes()}  # Inverti (x, y) per matplotlib

    nx.draw_networkx_nodes(topo_map, pos, nodelist=intersection_nodes, node_color='red', node_size=20, label='Intersezioni')
    nx.draw_networkx_nodes(topo_map, pos, nodelist=endpoint_nodes, node_color='blue', node_size=10, label='Punti Finali')
    nx.draw_networkx_edges(topo_map, pos, edge_color='green', width=1)
    
    plt.title("Mappa Topologica Sovrapposta alla Mappa Scheletrizzata")
    plt.axis('off')
    plt.show()

# Funzione principale
def main(image_path):
    # Carica la mappa scheletrizzata
    skeleton = load_map(image_path)
    
    # Crea la mappa binaria
    _, binary_skeleton = cv2.threshold(skeleton, 127, 1, cv2.THRESH_BINARY)
    
    # Genera la mappa delle distanze
    distance_map = generate_distance_map(binary_skeleton)
    
    # Crea il Diagramma di Voronoi
    voronoi_skeleton = generate_voronoi_diagram(distance_map)
    
    # Crea il grafo topologico
    topo_map = create_topological_graph(voronoi_skeleton)
    
    # Aggiungi archi al grafo
    topo_map = add_edges(topo_map, binary_skeleton)
    
    # Accorpa i nodi vicini
    topo_map = merge_close_nodes(topo_map, threshold=15)
    
    # Visualizza la mappa topologica
    visualize_topological_map(skeleton, topo_map)

# Esegui il codice
if __name__ == "__main__":
    image_path = '/path_to_your_map/skeleton_map.pgm'  # Modifica con il percorso della tua immagine
    main(image_path)
