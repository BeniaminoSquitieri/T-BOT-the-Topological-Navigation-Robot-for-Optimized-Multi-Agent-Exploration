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

# --- Funzione di verifica del percorso attraverso pixel bianchi ---

def check_line_passes_through_skeleton(node1, node2, skeleton):
    """
    Verifica se una linea tra due nodi passa interamente attraverso lo scheletro.

    Parameters:
        node1 (tuple): Coordinate (x, y) del primo nodo.
        node2 (tuple): Coordinate (x, y) del secondo nodo.
        skeleton (numpy.ndarray): L'immagine dello scheletro.

    Returns:
        bool: True se la linea passa interamente attraverso lo scheletro, False altrimenti.
    """
    x0, y0 = node1  # Coordinate del primo nodo
    x1, y1 = node2  # Coordinate del secondo nodo

    # Calcola il numero di punti lungo la linea
    num_points = max(abs(x1 - x0), abs(y1 - y0))
    if num_points == 0:
        num_points = 1  # Evita divisione per zero se i nodi coincidono

    # Genera valori di x e y lungo la linea tra i due nodi
    x_vals = np.linspace(x0, x1, num_points, dtype=int)
    y_vals = np.linspace(y0, y1, num_points, dtype=int)

    # Verifica che gli indici siano all'interno dei limiti dell'immagine
    valid_indices = (x_vals >= 0) & (x_vals < skeleton.shape[0]) & \
                    (y_vals >= 0) & (y_vals < skeleton.shape[1])
    x_vals = x_vals[valid_indices]
    y_vals = y_vals[valid_indices]

    # Verifica se tutti i punti lungo la linea appartengono allo scheletro
    if np.all(skeleton[x_vals, y_vals]):
        return True  # La linea passa interamente attraverso lo scheletro
    else:
        return False  # La linea non passa interamente attraverso lo scheletro

# --- Creazione del grafo topologico con nodi distribuiti uniformemente su X e Y ---

def create_topological_graph_using_skeleton(voronoi_skeleton, max_nodes=None):
    """
    Crea un grafo topologico basato sullo scheletro di Voronoi,
    con nodi distribuiti uniformemente sia lungo l'asse X che Y.

    Parameters:
        voronoi_skeleton (numpy.ndarray): L'immagine dello scheletro di Voronoi.
        max_nodes (int, optional): Numero massimo di nodi da inserire nel grafo.
                                   Se None, non c'è limite al numero di nodi.

    Returns:
        networkx.Graph: Il grafo topologico creato.
    """
    topo_map = ntx.Graph()  # Crea un nuovo grafo vuoto

    # Definisce un kernel per contare i vicini (esclude il pixel centrale)
    kernel = np.array([[1, 1, 1],
                       [1, 0, 1],
                       [1, 1, 1]], dtype=int)

    # Conta il numero di vicini per ogni pixel dello scheletro
    neighbor_count = convolve(voronoi_skeleton.astype(int), kernel, mode='constant', cval=0)

    # Identifica i nodi agli incroci o estremità (dove il numero di vicini è diverso da 2)
    node_positions = np.where((voronoi_skeleton == 1) & (neighbor_count != 2))

    # Crea una lista di coordinate dei nodi
    node_list = list(zip(node_positions[0], node_positions[1]))

    print(f"Numero di nodi identificati: {len(node_list)}")

    # Se non ci sono nodi, restituisce il grafo vuoto
    if len(node_list) == 0:
        print("Nessun nodo identificato nello scheletro.")
        return topo_map

    # Se non è specificato un limite massimo di nodi, utilizza tutti i nodi identificati
    if max_nodes is None:
        selected_nodes = node_list
    else:
        # Converti la lista di nodi in un array per facilità di manipolazione
        node_array = np.array(node_list)

        # Calcola il numero di celle della griglia in base a max_nodes
        num_cells = int(np.sqrt(max_nodes))
        if num_cells == 0:
            num_cells = 1  # Assicura almeno una cella

        # Ottieni i limiti dell'immagine
        rows, cols = voronoi_skeleton.shape

        # Calcola le dimensioni delle celle della griglia
        cell_height = rows / num_cells
        cell_width = cols / num_cells

        selected_nodes = []

        # Crea una griglia bidimensionale e seleziona un nodo per cella
        for i in range(num_cells):
            for j in range(num_cells):
                # Definisce i limiti della cella
                x_min = int(i * cell_height)
                x_max = int((i + 1) * cell_height)
                y_min = int(j * cell_width)
                y_max = int((j + 1) * cell_width)

                # Trova i nodi all'interno della cella
                nodes_in_cell = node_array[(node_array[:, 0] >= x_min) & (node_array[:, 0] < x_max) & \
                                           (node_array[:, 1] >= y_min) & (node_array[:, 1] < y_max)]

                # Se ci sono nodi nella cella, aggiungi uno di essi alla lista dei nodi selezionati
                if len(nodes_in_cell) > 0:
                    selected_nodes.append(tuple(nodes_in_cell[0]))  # Puoi anche scegliere un nodo casuale o centrale

                # Interrompe se abbiamo raggiunto il numero massimo di nodi
                if len(selected_nodes) >= max_nodes:
                    break
            else:
                continue
            break  # Esce dai cicli se il numero massimo di nodi è raggiunto

    # Aggiunge i nodi selezionati al grafo topologico
    topo_map.add_nodes_from(selected_nodes)

    # Aggiunge archi tra i nodi se esiste un percorso lungo lo scheletro
    for i in range(len(selected_nodes) - 1):
        for j in range(i + 1, len(selected_nodes)):
            # Verifica se esiste un percorso lungo lo scheletro tra i due nodi
            if check_line_passes_through_skeleton(selected_nodes[i], selected_nodes[j], voronoi_skeleton):
                topo_map.add_edge(selected_nodes[i], selected_nodes[j])

    # Stampa il numero totale di nodi e archi creati
    print(f"Numero totale di nodi creati: {len(topo_map.nodes())}")
    print(f"Numero totale di archi creati: {len(topo_map.edges())}")
    return topo_map  # Restituisce il grafo topologico

# --- Funzione di conversione dei nodi in waypoints ---

def convert_nodes_to_waypoints(topo_map, resolution, origin):
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
        # Converte le coordinate dei nodi in waypoints nel sistema di riferimento della mappa
        x_map = origin[0] + node[1] * resolution
        y_map = origin[1] + node[0] * resolution
        waypoints.append((x_map, y_map))
    return waypoints

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

# --- Funzione per salvare la mappa topologica con nodi in formato PGM ---

def save_topological_map_with_nodes(skeleton, topo_map, pgm_filename):
    """
    Sovrappone i nodi del grafo alla mappa scheletrizzata e salva l'immagine risultante.

    Parameters:
        skeleton (numpy.ndarray): L'immagine dello scheletro.
        topo_map (networkx.Graph): Il grafo topologico con i nodi.
        pgm_filename (str): Il percorso del file dove salvare l'immagine.
    """
    # Converti la mappa scheletrizzata in un'immagine in scala di grigi
    skeleton_with_nodes = (skeleton * 255).astype(np.uint8)

    # Converte le coordinate dei nodi per OpenCV (colonna, riga)
    node_positions = [(y, x) for x, y in topo_map.nodes()]
    for position in node_positions:
        # Disegna un cerchio per ogni nodo sulla mappa scheletrizzata
        cv2.circle(skeleton_with_nodes, position, 3, 255, -1)

    # Salva l'immagine con i nodi disegnati
    Image.fromarray(skeleton_with_nodes).save(pgm_filename)

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

# --- Funzione principale ---

def process_map(image_path, max_nodes=None):
    """
    Coordina tutti i passaggi necessari per processare la mappa e generare i file finali.

    Parameters:
        image_path (str): Il percorso dell'immagine della mappa da processare.
        max_nodes (int, optional): Numero massimo di nodi da inserire nel grafo topologico.
                                   Se None, non c'è limite al numero di nodi.
    """
    # Estrae il nome della mappa dall'immagine
    map_name = os.path.splitext(os.path.basename(image_path))[0]

    # Crea una cartella per questa mappa
    map_directory = create_map_directory(map_name+"_topological")

    # Passo 1: Carica la mappa di occupazione
    occupancy_grid = load_map(image_path)

    # Passo 2: Crea la mappa binaria
    binary_map = create_binary_map(occupancy_grid)
    # Salva la mappa binaria
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
    topo_map = create_topological_graph_using_skeleton(voronoi_skeleton, max_nodes=max_nodes)

    # Passo 7: Salva la mappa scheletrizzata con i nodi
    save_topological_map_with_nodes(voronoi_skeleton, topo_map,
                                    os.path.join(map_directory, f"{map_name}_topologica_scheletro_nodi.pgm"))

    # Passo 8: Converte i nodi del grafo in waypoints
    waypoints = convert_nodes_to_waypoints(topo_map, resolution=0.05, origin=(-32.507755, -27.073547, 0))

    # Passo 9: Salva i waypoints in un file YAML
    save_waypoints_as_yaml(waypoints, os.path.join(map_directory, f"{map_name}_waypoints.yaml"))

    # Passo 10: Salva il file YAML per ROS2
    save_as_yaml(os.path.join(map_directory, f"{map_name}_topologica_scheletro_nodi.yaml"),
                 f"{map_name}_topologica_scheletro_nodi.pgm")

# --- Esecuzione del codice ---

if __name__ == "__main__":
    # Crea il parser degli argomenti
    parser = argparse.ArgumentParser(description="Generazione di una mappa topologica da una mappa di occupazione.")
    parser.add_argument('image_path', type=str, help='Percorso dell\'immagine della mappa da processare.')
    parser.add_argument('--max_nodes', type=int, default=None, help='Numero massimo di nodi nel grafo topologico. Se non specificato, non c\'è limite.')

    # Parsea gli argomenti
    args = parser.parse_args()

    # Esegue il processo sulla mappa specificata
    process_map(args.image_path, max_nodes=args.max_nodes)