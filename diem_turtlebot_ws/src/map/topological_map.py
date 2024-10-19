import numpy as np
import cv2
import matplotlib.pyplot as plt
import networkx as ntx
from scipy.ndimage import distance_transform_edt
from skimage.morphology import skeletonize
from PIL import Image
import yaml

# --- Funzioni di base ---
def load_map(image_path):
    occupancy_grid = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if occupancy_grid is None:
        raise FileNotFoundError(f"Errore: impossibile aprire il file {image_path}")
    return occupancy_grid

def create_binary_map(occupancy_grid):
    _, binary_map = cv2.threshold(occupancy_grid, 240, 1, cv2.THRESH_BINARY)
    return binary_map

def compute_distance_map(binary_map):
    distance_map = distance_transform_edt(binary_map)
    return distance_map

def create_voronoi_lines(distance_map, binary_map):
    voronoi_map = np.zeros_like(binary_map)
    rows, cols = distance_map.shape
    for x in range(1, rows - 1):
        for y in range(1, cols - 1):
            neighbors = distance_map[x-1:x+2, y-1:y+2].flatten()
            if len(np.unique(neighbors)) > 1:
                voronoi_map[x, y] = 1
    return voronoi_map

def skeletonize_voronoi(voronoi_map):
    return skeletonize(voronoi_map)

# --- Funzione di verifica del percorso attraverso pixel bianchi ---
def check_line_passes_through_skeleton(node1, node2, skeleton):
    x0, y0 = node1
    x1, y1 = node2

    num_points = max(abs(x1 - x0), abs(y1 - y0))
    x_vals = np.linspace(x0, x1, num_points).astype(int)
    y_vals = np.linspace(y0, y1, num_points).astype(int)

    for x, y in zip(x_vals, y_vals):
        if skeleton[x, y] == 0:
            return False
    return True

# --- Creazione del grafo topologico con vincolo di archi solo sulle linee dello scheletro ---
def create_topological_graph_using_skeleton(voronoi_skeleton, max_nodes=50):
    rows, cols = voronoi_skeleton.shape
    topo_map = ntx.Graph()

    node_list = []
    for x in range(1, rows - 1):
        for y in range(1, cols - 1):
            if voronoi_skeleton[x, y]:
                neighbors = voronoi_skeleton[x-1:x+2, y-1:y+2].flatten()
                num_neighbors = np.sum(neighbors) - 1

                if num_neighbors > 2:
                    node_list.append((x, y))

    node_list = sorted(node_list, key=lambda node: node[1])
    segment_length = len(node_list) // max_nodes
    selected_nodes = []
    
    for i in range(0, len(node_list), segment_length):
        if len(selected_nodes) < max_nodes:
            selected_nodes.append(node_list[i])

    for node in selected_nodes:
        topo_map.add_node(node)

    for i in range(len(selected_nodes) - 1):
        for j in range(i + 1, len(selected_nodes)):
            if check_line_passes_through_skeleton(selected_nodes[i], selected_nodes[j], voronoi_skeleton):
                topo_map.add_edge(selected_nodes[i], selected_nodes[j])

    print(f"Numero totale di nodi creati: {len(topo_map.nodes())}")
    print(f"Numero totale di archi creati: {len(topo_map.edges())}")
    return topo_map

# --- Salvataggio e visualizzazione della mappa topologica ---
def save_topological_map(skeleton, topo_map, filename, title):
    plt.figure(figsize=(10, 10))
    plt.imshow(skeleton, cmap='gray')

    pos = {node: (node[1], node[0]) for node in topo_map.nodes()}
    ntx.draw_networkx_nodes(topo_map, pos, node_color='red', node_size=10)
    ntx.draw_networkx_edges(topo_map, pos, edge_color='orange', width=2)

    plt.title(title)
    plt.axis('off')
    plt.savefig(filename)
    plt.show()

# --- Funzioni per salvare la mappa nel formato pgm e yaml ---
def save_as_pgm(skeleton_map, pgm_filename):
    skeleton_map = (skeleton_map * 255).astype(np.uint8)
    Image.fromarray(skeleton_map).save(pgm_filename)

def save_as_yaml(yaml_filename, pgm_filename):
    data = {
        "image": f"./{pgm_filename}",
        "resolution": 0.050000,
        "origin": [-32.507755, -27.073547, 0.000000],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196
    }
    with open(yaml_filename, 'w') as file:
        yaml.dump(data, file)

# --- Funzione principale ---
def process_map(image_path, max_nodes=50):
    occupancy_grid = load_map(image_path)
    binary_map = create_binary_map(occupancy_grid)

    distance_map = compute_distance_map(binary_map)
    voronoi_map = create_voronoi_lines(distance_map, binary_map)
    voronoi_skeleton = skeletonize_voronoi(voronoi_map)

    topo_map = create_topological_graph_using_skeleton(voronoi_skeleton, max_nodes)

    # Salva la mappa topologica in formato PNG
    save_topological_map(voronoi_skeleton, topo_map, "mappa_topologica_scheletro.png", "Mappa Topologica con Arco sullo Scheletro")

    # Salva la mappa scheletrizzata in formato PGM
    save_as_pgm(voronoi_skeleton, "mappa_topologica_scheletro.pgm")

    # Salva il file YAML corrispondente
    save_as_yaml("mappa_topologica_scheletro.yaml", "mappa_topologica_scheletro.pgm")

# Esegui il codice
if __name__ == "__main__":
    image_path = 'diem_map.pgm'
    process_map(image_path, max_nodes=50)
