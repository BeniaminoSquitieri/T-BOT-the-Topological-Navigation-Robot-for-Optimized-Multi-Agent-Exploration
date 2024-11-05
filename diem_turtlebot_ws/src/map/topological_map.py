# Importazione delle librerie necessarie
import os  # Modulo per interagire con il sistema operativo
import numpy as np  # Libreria per operazioni numeriche avanzate su array
import cv2  # Libreria OpenCV per l'elaborazione delle immagini
import networkx as ntx  # Libreria per la creazione e manipolazione di grafi
from scipy.ndimage import distance_transform_edt, convolve, generic_filter  # Funzioni per elaborazione di immagini
from skimage.morphology import skeletonize  # Funzione per scheletrizzare immagini binarie
from PIL import Image  # Libreria per la manipolazione delle immagini
import yaml  # Libreria per leggere e scrivere file YAML
import argparse  # Modulo per gestire gli argomenti da riga di comando
from sklearn.cluster import DBSCAN
from scipy.spatial import cKDTree
from skimage.draw import line as draw_line
import logging
import json

class CoordinateTransformer:
    def __init__(self, image_height, resolution, origin):
        self.image_height = image_height
        self.resolution = resolution
        self.origin = origin  # (x_origin, y_origin)
    
    def pixel_to_map(self, node):
        y_pixel, x_pixel = node
        x_map = self.origin[0] + x_pixel * self.resolution
        y_map = self.origin[1] + (self.image_height - y_pixel) * self.resolution
        return x_map, y_map

    def map_to_pixel(self, x_map, y_map):
        x_pixel = int((x_map - self.origin[0]) / self.resolution)
        y_pixel = self.image_height - int((y_map - self.origin[1]) / self.resolution)
        return y_pixel, x_pixel

class Config:
    def __init__(self):
        self.resolution = 0.05  # metri per pixel
        self.origin = (-32.507755, -27.073547)  # (x_origin, y_origin)
        self.merge_threshold = 50  # in pixel
        self.max_connection_distance = 100000  # in pixel


# --- Funzioni di base ---

def load_map(image_path):
    """
    Carica la mappa di occupazione da un file immagine in scala di grigi.

    Parameters:
        image_path (str): Il percorso del file immagine da caricare.

    Returns:
        numpy.ndarray: L'immagine caricata in scala di grigi.
    """
    # Carica l'immagine in scala di grigi
    occupancy_grid = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if occupancy_grid is None:
        # Solleva un'eccezione se l'immagine non può essere caricata
        raise FileNotFoundError(f"Errore: impossibile aprire il file {image_path}")
    return occupancy_grid  # Restituisce l'immagine caricata

def clean_map(occupancy_grid):
    """
    Applica trasformazioni morfologiche per rappresentare la mappa in forma di rettangoli orizzontali e verticali.

    Parameters:
        occupancy_grid (numpy.ndarray): L'immagine in scala di grigi della mappa.

    Returns:
        numpy.ndarray: La mappa trasformata in rettangoli.
    """
    # Kernel rettangolare grande per la chiusura
    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (60, 60)) #Questa chiusura iniziale utilizza un kernel grande, ideale per unire piccoli spazi tra rettangoli.
    closed_map = cv2.morphologyEx(occupancy_grid, cv2.MORPH_CLOSE, kernel_close)

    # Dilatazione per ampliare i rettangoli, mantenendo direzioni preferenziali
    kernel_dilate = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))  # Più ampio in una direzione La dilatazione ha un kernel orientato più largo in una direzione, il che aiuta a espandere i rettangoli nelle direzioni orizzontale o verticale.
    dilated_map = cv2.dilate(closed_map, kernel_dilate, iterations=1)

    # Apertura per rimuovere piccole irregolarità e sporgenze
    kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (40, 65)) # L'apertura con un kernel più piccolo rimuove dettagli indesiderati lasciando intatte le forme più grandi.
    opened_map = cv2.morphologyEx(dilated_map, cv2.MORPH_OPEN, kernel_open)

    # Erosione finale per riportare alla forma originale mantenendo i rettangoli
    kernel_erode = cv2.getStructuringElement(cv2.MORPH_RECT, (30, 30)) # L'erosione finale aiuta a ripulire i bordi e ridurre la struttura alla sua dimensione rettangolare più coerente.
    final_map = cv2.erode(opened_map, kernel_erode, iterations=1)

    return final_map

def create_binary_map(occupancy_grid):
    """
    Converte l'immagine della mappa in una mappa binaria.

    Parameters:
        occupancy_grid (numpy.ndarray): L'immagine in scala di grigi della mappa.

    Returns:
        numpy.ndarray: La mappa binaria risultante.
    """
    # Utilizza una soglia per binarizzare l'immagine
    # I pixel con valore superiore a 240 diventano 1 (bianco), altrimenti 0 (nero)
    _, binary_map = cv2.threshold(occupancy_grid, 240, 1, cv2.THRESH_BINARY)
    return binary_map  # Restituisce la mappa binaria

def compute_distance_map(binary_map):
    """
    Calcola la mappa delle distanze euclidee dai pixel non zero.

    Parameters:
        binary_map (numpy.ndarray): La mappa binaria della mappa.

    Returns:
        numpy.ndarray: La mappa delle distanze euclidee.
    """
    # Calcola la mappa delle distanze euclidee
    distance_map = distance_transform_edt(binary_map)
    return distance_map  # Restituisce la mappa delle distanze

def create_voronoi_lines(distance_map):
    """
    Crea le linee di Voronoi identificando i pixel con vicini di valori di distanza diversi.

    Parameters:
        distance_map (numpy.ndarray): La mappa delle distanze euclidee.

    Returns:
        numpy.ndarray: La mappa di Voronoi risultante.
    """
    # Definisce un kernel booleano per escludere il pixel centrale
    kernel = np.array([[1, 1, 1],
                       [1, 0, 1],
                       [1, 1, 1]], dtype=bool)

    # Definisce una funzione che calcola la differenza tra il valore massimo e minimo nella finestra
    def local_range(values):
        return values.max() - values.min()

    # Applica la funzione local_range su ogni pixel utilizzando il kernel
    local_ranges = generic_filter(distance_map, local_range, footprint=kernel)
    # Crea la mappa di Voronoi dove la differenza locale è maggiore di zero
    voronoi_map = (local_ranges > 0).astype(np.uint8)
    # Imposta i bordi a zero per coerenza con il metodo originale
    voronoi_map[0, :] = 0
    voronoi_map[-1, :] = 0
    voronoi_map[:, 0] = 0
    voronoi_map[:, -1] = 0

    return voronoi_map  # Restituisce la mappa di Voronoi

def skeletonize_voronoi(voronoi_map):
    """
    Scheletrizza la mappa di Voronoi per ottenere linee sottili.

    Parameters:
        voronoi_map (numpy.ndarray): La mappa di Voronoi.

    Returns:
        numpy.ndarray: L'immagine scheletrizzata della mappa di Voronoi.
    """
    # Scheletrizza la mappa di Voronoi
    return skeletonize(voronoi_map)

def convert_to_map_coordinates(node, image_height, resolution, origin):
    """
    Converte le coordinate del nodo da pixel a coordinate mappa con inversione dell'asse y.

    Parameters:
        node (tuple): Coordinate del nodo in pixel (y, x).
        image_height (int): Altezza dell'immagine in pixel.
        resolution (float): La risoluzione della mappa (metri per pixel).
        origin (tuple): L'origine della mappa (x, y).

    Returns:
        tuple: Coordinate (x_map, y_map) nel sistema di riferimento della mappa.
    """
    y_pixel, x_pixel = node  # Nota che l'ordine è (y, x)

    # Invertiamo l'asse Y: usiamo (image_height - y_pixel) per la conversione
    x_map = origin[0] + x_pixel * resolution
    y_map = origin[1] + (image_height - y_pixel) * resolution

    return x_map, y_map

# --- Funzione di verifica del percorso attraverso pixel bianchi ---
def create_topological_graph_using_skeleton(voronoi_skeleton, max_nodes=None, merge_threshold=50, max_connection_distance=100000, resolution=0.05, origin=(-32.507755, -27.073547), image_height=None):
    """
    Crea un grafo topologico basato sullo scheletro di Voronoi,
    con nodi distribuiti uniformemente sia lungo l'asse X che Y.
    I nodi vicini entro una certa soglia vengono fusi.
    """
    topo_map = ntx.Graph()
    
    # Definisce un kernel per contare i vicini (esclude il pixel centrale)
    kernel = np.array([[1, 1, 1],
                       [1, 0, 1],
                       [1, 1, 1]], dtype=int)
    
    # Conta il numero di vicini per ogni pixel dello scheletro
    neighbor_count = convolve(voronoi_skeleton.astype(int), kernel, mode='constant', cval=0)
    
    # Identifica i nodi agli incroci o estremità (dove il numero di vicini è diverso da 2)
    node_positions = np.column_stack(np.where((voronoi_skeleton == 1) & (neighbor_count != 2)))
    
    # Applica DBSCAN per fondere i nodi vicini
    clustering = DBSCAN(eps=merge_threshold, min_samples=1).fit(node_positions)
    
    # Calcola il centroide di ciascun cluster per ottenere i nodi fusi
    fused_nodes = []
    for cluster_id in np.unique(clustering.labels_):
        cluster_points = node_positions[clustering.labels_ == cluster_id]
        centroid = np.mean(cluster_points, axis=0).astype(int)
        fused_nodes.append(tuple(centroid))
    
    logging.info("Fusione dei nodi completata. Numero di nodi dopo la fusione: %d", len(fused_nodes))

    
    # Aggiunge i nodi fusi al grafo
    topo_map.add_nodes_from(fused_nodes)
    
    # Costruisce un KD-Tree per cercare nodi vicini efficientemente
    node_tree = cKDTree(fused_nodes)
    
    # Trova coppie di nodi entro la distanza massima
    pairs = node_tree.query_pairs(r=max_connection_distance)
    
    # Procedura per la creazione degli archi
    for i, j in pairs:
        node_i = fused_nodes[i]
        node_j = fused_nodes[j]
        if check_line_passes_through_skeleton(node_i, node_j, voronoi_skeleton):
            topo_map.add_edge(node_i, node_j)
            # Debug opzionale
            # print(f"Arco creato tra {node_i} e {node_j}")
    
    return topo_map


# Funzione di controllo del percorso attraverso lo scheletro
def check_line_passes_through_skeleton(node1, node2, skeleton):
    """
    Verifica se una linea tra due nodi passa interamente attraverso lo scheletro.
    """
    y0, x0 = node1  # Coordinate del primo nodo
    y1, x1 = node2  # Coordinate del secondo nodo

    # Usa l'algoritmo di Bresenham per ottenere i pixel lungo la linea
    rr, cc = draw_line(y0, x0, y1, x1)

    # Verifica che gli indici siano all'interno dei limiti dell'immagine
    valid_indices = (rr >= 0) & (rr < skeleton.shape[0]) & \
                    (cc >= 0) & (cc < skeleton.shape[1])
    rr = rr[valid_indices]
    cc = cc[valid_indices]

    # Verifica se tutti i punti lungo la linea appartengono allo scheletro
    return np.all(skeleton[rr, cc] == 1)


# --- Funzione per convertire i valori in formati Python standard ---
def numpy_to_python(obj):
    """
    Converte i tipi NumPy in tipi Python standard.
    Parameters:
        obj: Oggetto NumPy (array o valore).
    Returns:
        Oggetto convertito in un tipo standard Python (float o int).
    """
    if isinstance(obj, np.ndarray):
        return obj.tolist()  # Se è un array NumPy, lo converte in lista
    elif isinstance(obj, np.generic):
        return obj.item()  # Se è un oggetto NumPy generico, lo converte in un valore Python
    return obj

# --- Funzione per salvare i waypoints con label in un file YAML ---

def save_waypoints_as_yaml(waypoints, filename):
    """
    Salva i waypoints con label in un file YAML.

    Parameters:
        waypoints (list): Lista dei waypoints (x, y) da salvare.
        filename (str): Il percorso del file YAML in cui salvare i waypoints.
    """
    # Crea un dizionario per i dati dei waypoints, assegnando un label a ciascun nodo
    waypoints_data = {"waypoints": [{"label": f"node_{i+1}", "x": numpy_to_python(wp[0]), "y": numpy_to_python(wp[1])} 
                                    for i, wp in enumerate(waypoints)]}

    # Salva il dizionario in formato YAML
    with open(filename, 'w') as yaml_file:
        yaml.dump(waypoints_data, yaml_file, default_flow_style=False)

# --- Funzione per salvare un'immagine PGM ---

def save_as_pgm(image, filename):
    """
    Salva un'immagine numpy array in formato PGM.

    Parameters:
        image (numpy.ndarray): L'immagine da salvare.
        filename (str): Il percorso del file dove salvare l'immagine.
    """
    # Converte l'immagine in uint8 e scala i valori da 0 a 255 se necessario
    Image.fromarray((image * 255).astype(np.uint8)).save(filename)


def save_graph_as_json(topo_map, filename, transformer):
    """
    Salva il grafo topologico con i nodi e archi in formato JSON.

    Parameters:
        topo_map (networkx.Graph): Il grafo topologico con i nodi e archi.
        filename (str): Il percorso del file JSON in cui salvare il grafo.
        transformer (CoordinateTransformer): Oggetto per trasformare le coordinate pixel in coordinate mappa.
    """
    # Convert nodes with map coordinates
    nodes = []
    for i, node in enumerate(topo_map.nodes()):
        x_map, y_map = transformer.pixel_to_map(node)
        nodes.append({"label": f"node_{i+1}", "x": x_map, "y": y_map})

    # Convert edges
    edges = [{"source": f"node_{i+1}", "target": f"node_{j+1}"} for i, j in topo_map.edges()]

    # Structure the data in JSON format
    graph_data = {
        "nodes": nodes,
        "edges": edges
    }

    # Save to JSON file
    with open(filename, 'w') as json_file:
        json.dump(graph_data, json_file, indent=4)
    
    logging.info(f"Grafo topologico salvato in formato JSON in {filename}")


# --- Funzione per salvare la mappa topologica con nodi in formato PGM ---0
def save_topological_map_with_nodes(skeleton, topo_map, pgm_filename, transformer):
    """
    Sovrappone i nodi del grafo alla mappa scheletrizzata, disegnando le coordinate di ciascun nodo sull'immagine,
    e salva l'immagine risultante in formato PGM.
    """
    # Inverti l'immagine dello scheletro per migliorare la visibilità
    skeleton_with_nodes = 255 - (skeleton * 255).astype(np.uint8)

    # Colore e font per i cerchi e il testo
    node_color = 0  # Colore nero per i nodi
    text_color = (0)  # Colore nero per il testo
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.6
    thickness = 2

    # Disegna i nodi usando le coordinate in pixel
    for node in topo_map.nodes():
        y_pixel, x_pixel = node
        x_map, y_map = transformer.pixel_to_map(node)
        coordinate_text = f"({x_map:.2f}, {y_map:.2f})"

        # Disegna il nodo come cerchio più grande
        cv2.circle(skeleton_with_nodes, (x_pixel, y_pixel), 6, node_color, -1)

        # Aggiungi il testo con le coordinate in metri sopra il nodo
        cv2.putText(skeleton_with_nodes, coordinate_text, (x_pixel + 5, y_pixel - 5), font, font_scale, text_color, thickness)

        # Stampa per debug (coordinate in pixel e coordinate mappa)
        # logging.info(f"Nodo salvato: Coordinate pixel ({x_pixel}, {y_pixel}) -> Coordinate mappa {coordinate_text}")

    # Salva l'immagine con i nodi e le coordinate disegnate
    Image.fromarray(skeleton_with_nodes).save(pgm_filename)
    logging.info(f"Mappa con coordinate dei nodi salvata in {pgm_filename}")



# --- Funzione per salvare il file YAML associato ---

def save_as_yaml(yaml_filename, pgm_filename):
    """
    Crea un file YAML che contiene le informazioni necessarie per utilizzare la mappa in ROS2.

    Parameters:
        yaml_filename (str): Il percorso del file YAML da creare.
        pgm_filename (str): Il nome del file PGM della mappa.
    """
    # Crea un dizionario con i dati necessari per il file YAML
    yaml_data = {
        "image": f"./{pgm_filename}",  # Percorso relativo all'immagine della mappa
        "resolution": 0.050000,  # Risoluzione della mappa (ad esempio, 5 cm per pixel)
        "origin": [-32.507755, -27.073547, 0.000000],  # Coordinate di origine della mappa
        "negate": 0,  # Indica se invertire i colori dell'immagine
        "occupied_thresh": 0.65,  # Soglia per considerare un pixel come occupato
        "free_thresh": 0.196  # Soglia per considerare un pixel come libero
    }
    # Scrive il dizionario nel file YAML
    with open(yaml_filename, 'w') as file:
        yaml.dump(yaml_data, file, default_flow_style=False)

# --- Funzione per creare una cartella specifica per la mappa ---

def create_map_directory(map_name):
    """
    Assicura che esista una cartella con il nome della mappa.

    Parameters:
        map_name (str): Il nome della mappa e della cartella.

    Returns:
        str: Il nome della directory creata o esistente.
    """
    # Crea una directory con il nome della mappa se non esiste già
    if not os.path.exists(map_name):
        os.makedirs(map_name)
    return map_name  # Restituisce il nome della directory

def save_pixel_to_map_transformations(topo_map, filename, transformer):
    """
    Salva la trasformazione tra coordinate pixel e coordinate mappa in un file di testo.

    Parameters:
        topo_map (networkx.Graph): Il grafo topologico con i nodi.
        filename (str): Il percorso del file di testo in cui salvare le trasformazioni.
        transformer (CoordinateTransformer): L'oggetto per trasformare le coordinate.
    """
    with open(filename, 'w') as file:
        file.write("Transformazione tra coordinate pixel e coordinate mappa:\n")
        file.write("Formato: (Pixel X, Pixel Y) -> (Mappa X, Mappa Y)\n\n")

        for node in topo_map.nodes():
            # Coordinate in pixel del nodo
            y_pixel, x_pixel = node

            # Conversione alle coordinate mappa
            x_map, y_map = transformer.pixel_to_map(node)

            # Scrivi la trasformazione nel file
            file.write(f"Pixel ({x_pixel}, {y_pixel}) -> Mappa ({x_map:.2f}, {y_map:.2f})\n")

    logging.info(f"Transformazione salvata in {filename}")

        
# --- Funzione di conversione dei nodi in waypoints ---
def convert_nodes_to_waypoints(topo_map, transformer):
    """
    Converte i nodi del grafo in waypoints nel sistema di riferimento della mappa.

    Parameters:
        topo_map (networkx.Graph): Il grafo topologico con i nodi.
        resolution (float): La risoluzione della mappa (metri per pixel).
        origin (tuple): L'origine della mappa nel sistema di riferimento (x, y, theta).

    Returns:
        list: Una lista di waypoints (x, y) nel sistema di riferimento della mappa.
    """
    waypoints = []
    for node in topo_map.nodes():
        x_map, y_map = transformer.pixel_to_map(node)
        waypoints.append((x_map, y_map))
    return waypoints


# --- Funzione principale ---
def process_map(image_path, max_nodes=None):
    """
    Coordina tutti i passaggi necessari per processare la mappa e generare i file finali.

    Parameters:
        image_path (str): Il percorso dell'immagine della mappa da processare.
        max_nodes (int, optional): Numero massimo di nodi da inserire nel grafo topologico.
                                   Se None, non c'è limite al numero di nodi.
    """
    config = Config()
    # Estrae il nome della mappa dall'immagine
    map_name = os.path.splitext(os.path.basename(image_path))[0]

    # Crea una cartella per questa mappa
    map_directory = create_map_directory(map_name + "_topological")

    # Passo 1: Carica e pulisce la mappa di occupazione
    occupancy_grid = load_map(image_path)
    image_height = occupancy_grid.shape[0]  # Calcola l'altezza dell'immagine in pixel

    # Crea un oggetto CoordinateTransformer
    transformer = CoordinateTransformer(image_height, config.resolution, config.origin)

    cleaned_map = clean_map(occupancy_grid)  # Applica la pulizia della mappa
    save_as_pgm(cleaned_map, os.path.join(map_directory, f"{map_name}_cleaned_map.pgm"))

    # Passo 2: Crea la mappa binaria
    binary_map = create_binary_map(cleaned_map)  # Usa la mappa pulita come input
    save_as_pgm(binary_map, os.path.join(map_directory, f"{map_name}_binary_map.pgm"))

    # Passo 3: Calcola la mappa delle distanze euclidee
    distance_map = compute_distance_map(binary_map)
    # Normalizza la mappa delle distanze per la visualizzazione
    distance_map_normalized = (distance_map / np.max(distance_map) * 255).astype(np.uint8)
    # Salva la mappa delle distanze normalizzata
    save_as_pgm(distance_map_normalized, os.path.join(map_directory, f"{map_name}_distance_map.pgm"))

    # Passo 4: Crea le linee di Voronoi
    voronoi_map = create_voronoi_lines(distance_map)
    # Salva la mappa di Voronoi
    save_as_pgm(voronoi_map, os.path.join(map_directory, f"{map_name}_voronoi_map.pgm"))

    # Passo 5: Scheletrizza le linee di Voronoi
    voronoi_skeleton = skeletonize_voronoi(voronoi_map)
    # Salva lo scheletro di Voronoi
    save_as_pgm(voronoi_skeleton, os.path.join(map_directory, f"{map_name}_skeleton_voronoi.pgm"))

    # Passo 6: Creazione del grafo topologico utilizzando lo scheletro
    topo_map = create_topological_graph_using_skeleton(
        voronoi_skeleton,
        max_nodes=max_nodes,
        merge_threshold=config.merge_threshold,
        max_connection_distance=config.max_connection_distance,
        resolution=config.resolution,
        origin=config.origin,
        image_height=image_height
    )

    # Passo 7: Salva il grafo topologico come JSON
    save_graph_as_json(topo_map, os.path.join(map_directory, f"{map_name}_topological_graph.json"), transformer)

    # Salva la trasformazione pixel-mappa in un file txt
    save_pixel_to_map_transformations(
        topo_map,
        os.path.join(map_directory, f"{map_name}_pixel_to_map_transformations.txt"),
        transformer
    )

    # Passo 8: Salva la mappa scheletrizzata con i nodi
    save_topological_map_with_nodes(
        voronoi_skeleton,
        topo_map,
        os.path.join(map_directory, f"{map_name}_topologica_scheletro_nodi.pgm"),
        transformer
    )



### UBUNTU VERSION
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    # Percorso di default per la mappa
    default_image_path = "/home/beniamino/turtlebot4/diem_turtlebot_ws/src/map/diem_map.pgm"

    # Crea il parser degli argomenti
    parser = argparse.ArgumentParser(description="Generazione di una mappa topologica da una mappa di occupazione.")
    parser.add_argument('image_path', type=str, nargs='?', default=default_image_path,
                        help="Percorso dell'immagine della mappa da processare (predefinito: diem_turtlebot_ws/src/map/diem_map.pgm)")
    parser.add_argument('--max_nodes', type=int, default=None, help="Numero massimo di nodi nel grafo topologico. Se non specificato, non c'è limite.")

    # Parsea gli argomenti
    args = parser.parse_args()

    # Esegue il processo sulla mappa specificata
    process_map(args.image_path, max_nodes=args.max_nodes)



# #### WINDOWS  VERSION

# if __name__ == "__main__":
#     logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
#     # Percorso di default per la mappa, con doppie backslash per Windows
#     default_image_path = "diem_map.pgm"

#     # Crea il parser degli argomenti
#     parser = argparse.ArgumentParser(description="Generazione di una mappa topologica da una mappa di occupazione.")
#     parser.add_argument('image_path', type=str, nargs='?', default=default_image_path,
#                         help="Percorso dell'immagine della mappa da processare (predefinito: diem_turtlebot_ws/src/map/diem_map.pgm)")
#     parser.add_argument('--max_nodes', type=int, default=None, help="Numero massimo di nodi nel grafo topologico. Se non specificato, non c'è limite.")

#     # Parsea gli argomenti
#     args = parser.parse_args()

#     # Esegue il processo sulla mappa specificata
#     process_map(args.image_path, max_nodes=args.max_nodes)
