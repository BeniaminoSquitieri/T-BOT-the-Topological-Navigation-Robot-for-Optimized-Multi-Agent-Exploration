import numpy as np
import cv2
import matplotlib.pyplot as plt
import networkx as nx
from scipy.ndimage import distance_transform_edt
from skimage.morphology import skeletonize
from scipy.spatial import distance

# --- Funzione per caricare la mappa ---
def load_map(image_path):
    """
    Carica un'immagine in scala di grigi dal percorso specificato.

    Parameters:
        image_path (str): Il percorso dell'immagine.

    Returns:
        numpy.ndarray: L'immagine caricata in scala di grigi.
    """
    # Carica l'immagine in scala di grigi
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"Errore: impossibile aprire il file {image_path}")
    return image

# --- Funzione per generare la mappa delle distanze euclidee ---
def generate_distance_map(binary_map):
    """
    Calcola la mappa delle distanze euclidee invertendo la mappa binaria.

    Parameters:
        binary_map (numpy.ndarray): Mappa binaria dell'immagine.

    Returns:
        numpy.ndarray: Mappa delle distanze euclidee.
    """
    # Calcola la mappa delle distanze euclidee
    distance_map = distance_transform_edt(1 - binary_map)
    return distance_map

# --- Funzione per creare il diagramma di Voronoi (scheletrizzazione) ---
def generate_voronoi_diagram(distance_map):
    """
    Genera lo scheletro della mappa utilizzando la scheletrizzazione.

    Parameters:
        distance_map (numpy.ndarray): Mappa delle distanze euclidee.

    Returns:
        numpy.ndarray: Mappa scheletrizzata (scheletro di Voronoi).
    """
    # Applica una soglia alla mappa delle distanze
    binary_distance_map = distance_map > 0
    # Applica la scheletrizzazione
    skeleton = skeletonize(binary_distance_map).astype(np.uint8)
    return skeleton

def create_topological_graph(voronoi_skeleton):
    """
    Crea un grafo topologico a partire dallo scheletro di Voronoi.

    Parameters:
        voronoi_skeleton (numpy.ndarray): Mappa scheletrizzata.

    Returns:
        networkx.Graph: Grafo topologico creato.
    """
    # Inizializza il grafo
    topo_map = nx.Graph()

    # Trova i pixel dello scheletro
    skeleton_points = np.argwhere(voronoi_skeleton == 1)

    # Definisce i possibili movimenti (8 connettività)
    moves = [(-1, -1), (-1, 0), (-1, 1),
             (0, -1),          (0, 1),
             (1, -1),  (1, 0),  (1, 1)]

    # Per ogni punto dello scheletro
    for point in skeleton_points:
        x, y = point
        # Aggiungi il nodo al grafo
        topo_map.add_node((x, y))
        # Cerca i vicini
        for dx, dy in moves:
            neighbor_x, neighbor_y = x + dx, y + dy
            # Verifica che le coordinate siano all'interno dei limiti dell'immagine
            if (0 <= neighbor_x < voronoi_skeleton.shape[0]) and (0 <= neighbor_y < voronoi_skeleton.shape[1]):
                if voronoi_skeleton[neighbor_x, neighbor_y] == 1:
                    topo_map.add_edge((x, y), (neighbor_x, neighbor_y))

    return topo_map


# --- Funzione per semplificare il grafo rimuovendo i nodi intermedi ---
def simplify_graph(topo_map):
    """
    Semplifica il grafo rimuovendo i nodi con grado 2 e consolidando gli archi.

    Parameters:
        topo_map (networkx.Graph): Grafo topologico da semplificare.

    Returns:
        networkx.Graph: Grafo topologico semplificato.
    """
    simplified_map = topo_map.copy()
    nodes_to_remove = []

    for node in topo_map.nodes():
        # Se il nodo ha grado 2
        if topo_map.degree(node) == 2:
            neighbors = list(topo_map.neighbors(node))
            # Controlla se i vicini non sono lo stesso nodo
            if neighbors[0] != neighbors[1]:
                # Aggiungi un arco diretto tra i due vicini
                simplified_map.add_edge(neighbors[0], neighbors[1])
            # Marca il nodo per la rimozione
            nodes_to_remove.append(node)

    # Rimuove i nodi intermedi
    simplified_map.remove_nodes_from(nodes_to_remove)

    return simplified_map

# --- Funzione per visualizzare la mappa topologica ---
def visualize_topological_map(voronoi_skeleton, topo_map):
    """
    Visualizza la mappa scheletrizzata con il grafo topologico sovrapposto.

    Parameters:
        voronoi_skeleton (numpy.ndarray): Mappa scheletrizzata.
        topo_map (networkx.Graph): Grafo topologico.
    """
    plt.figure(figsize=(10, 10))
    plt.imshow(voronoi_skeleton, cmap='gray')

    pos = {node: (node[1], node[0]) for node in topo_map.nodes()}
    nx.draw(topo_map, pos, node_size=10, node_color='red', edge_color='blue', width=1)

    plt.title("Mappa Topologica Sovrapposta allo Scheletro di Voronoi")
    plt.axis('off')
    plt.show()

# --- Funzione principale ---
def main(image_path):
    # Carica l'immagine della mappa
    occupancy_map = load_map(image_path)

    # Binarizza l'immagine (0: ostacolo, 1: libero)
    _, binary_map = cv2.threshold(occupancy_map, 240, 1, cv2.THRESH_BINARY_INV)

    # Genera la mappa delle distanze euclidee
    distance_map = generate_distance_map(binary_map)

    # Genera lo scheletro di Voronoi
    voronoi_skeleton = generate_voronoi_diagram(distance_map)

    # Crea il grafo topologico
    topo_map = create_topological_graph(voronoi_skeleton)

    # Semplifica il grafo rimuovendo i nodi intermedi
    simplified_topo_map = simplify_graph(topo_map)

    # Visualizza la mappa topologica
    visualize_topological_map(voronoi_skeleton, simplified_topo_map)

# Esegui il codice
if __name__ == "__main__":
    image_path = 'diem_map.pgm'  # Modifica con il percorso della tua immagine
    main(image_path)
