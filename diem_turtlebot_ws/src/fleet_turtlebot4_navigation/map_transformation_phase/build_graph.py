#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse          # Per la gestione degli argomenti da riga di comando
import json              # Per leggere e scrivere file JSON
import cv2               # OpenCV, per l'elaborazione delle immagini
import numpy as np       # NumPy, per operazioni numeriche efficienti su array
import yaml              # Per leggere file di configurazione in formato YAML
import math              # Per operazioni matematiche di base
import os                # Per interagire con il sistema operativo (es. percorsi di file)

# Importa la funzione 'line' di Bresenham dalla libreria scikit-image
from skimage.draw import line

# Prova a importare KDTree da scikit-learn per l'efficiente ricerca dei vicini
try:
    from sklearn.neighbors import KDTree
    USE_SKLEARN = True  # Flag per indicare se sklearn è disponibile
except ImportError:
    # Se sklearn non è installato, imposta il flag a False e utilizza un metodo alternativo
    USE_SKLEARN = False

# Importa NetworkX per la gestione e manipolazione dei grafi
import networkx as nx

###############################################################################
# Funzioni di supporto
###############################################################################

def load_map_info(yaml_file):
    """
    Carica le informazioni di mappa da un file .yaml.

    Parametri:
        yaml_file (str): Percorso al file YAML contenente le informazioni della mappa.

    Ritorna:
        dict: Dizionario contenente le informazioni della mappa, inclusi percorso immagine,
              risoluzione, origine, e soglie per pixel occupati/liberi.
    """
    # Apre e legge il file YAML
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)  # Carica i dati YAML in un dizionario Python

    map_info = {}  # Dizionario per memorizzare le informazioni della mappa

    # Determina la directory del file YAML per gestire percorsi relativi
    yaml_dir = os.path.dirname(os.path.abspath(yaml_file))

    # Estrae il percorso relativo dell'immagine dal file YAML
    image_rel_path = data["image"]  # Esempio: './diem_map.pgm'

    # Costruisce il percorso assoluto dell'immagine combinando la directory YAML e il percorso relativo
    image_path = os.path.join(yaml_dir, image_rel_path)

    # Popola il dizionario map_info con le informazioni rilevanti
    map_info["image_path"] = image_path
    map_info["resolution"] = float(data["resolution"])  # Risoluzione in metri/pixel
    map_info["origin"] = data["origin"]  # Origine della mappa: [origin_x, origin_y, origin_theta]
    map_info["negate"] = int(data.get("negate", 0))  # Flag per invertire i pixel (0 o 1)
    map_info["occupied_thresh"] = float(data.get("occupied_thresh", 0.65))  # Soglia per pixel occupati
    map_info["free_thresh"] = float(data.get("free_thresh", 0.196))  # Soglia per pixel liberi

    return map_info  # Restituisce il dizionario con le informazioni della mappa


def load_occupancy_image(map_info):
    """
    Carica la mappa di occupazione in scala di grigi utilizzando OpenCV.

    Parametri:
        map_info (dict): Dizionario contenente le informazioni della mappa.

    Ritorna:
        tuple: Una tupla contenente l'immagine caricata (numpy.ndarray),
               l'altezza (int) e la larghezza (int) dell'immagine.
    """
    # Carica l'immagine usando OpenCV in modalità non modificata
    img = cv2.imread(map_info["image_path"], cv2.IMREAD_UNCHANGED)

    # Se l'immagine non viene caricata correttamente, solleva un'eccezione
    if img is None:
        raise FileNotFoundError(f"Impossibile caricare l'immagine: {map_info['image_path']}")

    # Se il flag 'negate' è impostato a 1, inverte i valori dei pixel
    if map_info["negate"] == 1:
        img = 255 - img  # Inversione dei pixel (bianco diventa nero e viceversa)

    # Ottiene le dimensioni dell'immagine (altezza e larghezza)
    height, width = img.shape

    return img, height, width  # Restituisce l'immagine e le sue dimensioni


def load_nodes(json_file):
    """
    Carica i nodi (waypoints) da un file JSON.

    La struttura attesa del file JSON:
    {
      "nodes": [
        { "label": "node_1", "x": ..., "y": ... },
        ...
      ]
    }

    Parametri:
        json_file (str): Percorso al file JSON contenente i nodi.

    Ritorna:
        list: Lista di tuple, ognuna contenente (label, x, y) per ogni nodo.
    """
    # Apre e legge il file JSON
    with open(json_file, 'r') as f:
        data = json.load(f)  # Carica i dati JSON in un dizionario Python

    nodes_data = data["nodes"]  # Estrae la lista dei nodi

    nodes_list = []  # Lista per memorizzare i nodi come tuple

    # Itera su ogni nodo nella lista
    for nd in nodes_data:
        label = nd["label"]  # Etichetta del nodo (es. "node_1")
        x = nd["x"]          # Coordinata x nel sistema di riferimento del mondo
        y = nd["y"]          # Coordinata y nel sistema di riferimento del mondo
        nodes_list.append((label, x, y))  # Aggiunge la tupla alla lista

    return nodes_list  # Restituisce la lista dei nodi


def world_to_map(x_world, y_world, map_info, map_height):
    """
    Converte coordinate del mondo (x_world, y_world) in coordinate di pixel (row, col) dell'immagine.

    Assume che:
      - origin_x e origin_y siano in basso a sinistra dell'immagine
      - la risoluzione sia in metri/pixel
      - l'immagine in OpenCV abbia l'origine (0,0) in alto a sinistra

    Formula di conversione:
      col = int( (x_world - origin_x) / resolution )
      row = int( map_height - 1 - (y_world - origin_y) / resolution )

    Parametri:
        x_world (float): Coordinata x nel sistema di riferimento del mondo.
        y_world (float): Coordinata y nel sistema di riferimento del mondo.
        map_info (dict): Dizionario contenente le informazioni della mappa.
        map_height (int): Altezza dell'immagine in pixel.

    Ritorna:
        tuple: Coordinate di pixel (row, col).
    """
    # Estrae le coordinate di origine e la risoluzione dal dizionario map_info
    origin_x, origin_y, _ = map_info["origin"]
    resolution = map_info["resolution"]

    # Calcola le coordinate in pixel come float
    col_f = (x_world - origin_x) / resolution  # Coordinata colonna
    row_f = (y_world - origin_y) / resolution  # Coordinata riga

    # Converte le coordinate float in interi usando floor
    col = int(math.floor(col_f))
    row = int(math.floor(map_height - 1 - row_f))  # Inverte l'asse y per OpenCV

    return row, col  # Restituisce la tupla (row, col)


def is_free_pixel(img, row, col, free_value_threshold):
    """
    Verifica se un pixel specificato è considerato 'libero' o 'occupato'.

    Parametri:
        img (numpy.ndarray): Immagine della mappa in scala di grigi.
        row (int): Indice della riga del pixel.
        col (int): Indice della colonna del pixel.
        free_value_threshold (float): Soglia per determinare se un pixel è libero.

    Ritorna:
        bool: True se il pixel è libero, False altrimenti.
    """
    h, w = img.shape  # Ottiene le dimensioni dell'immagine

    # Verifica se le coordinate sono fuori dai limiti dell'immagine
    if row < 0 or row >= h or col < 0 or col >= w:
        return False  # Fuori dai limiti => considerato occupato

    # Verifica se il valore del pixel è maggiore o uguale alla soglia
    return (img[row, col] >= free_value_threshold)


def check_collision(img, r1, c1, r2, c2, free_value_threshold=50, collision_tolerance=1.0):
    """
    Verifica se il segmento tra due pixel attraversa aree occupate della mappa.

    Utilizza l'algoritmo di Bresenham per tracciare la linea tra (r1, c1) e (r2, c2).
    Se la proporzione di pixel liberi lungo la linea è >= collision_tolerance, 
    allora non c'è collisione. Altrimenti, c'è collisione.

    Parametri:
        img (numpy.ndarray): Immagine della mappa in scala di grigi.
        r1 (int): Riga di partenza.
        c1 (int): Colonna di partenza.
        r2 (int): Riga di arrivo.
        c2 (int): Colonna di arrivo.
        free_value_threshold (float, optional): Soglia per considerare un pixel libero. Default è 50.
        collision_tolerance (float, optional): Percentuale minima di pixel liberi per considerare il segmento sgombro. Default è 1.0 (100%).

    Ritorna:
        bool: True se c'è collisione, False se non c'è collisione.
    """
    # Traccia la linea tra i due punti usando l'algoritmo di Bresenham
    rr, cc = line(r1, c1, r2, c2)  # Liste di righe e colonne dei pixel lungo la linea

    total_pixels = len(rr)  # Numero totale di pixel lungo la linea
    free_pixels = 0         # Contatore dei pixel liberi

    # Itera su ogni pixel lungo la linea
    for i in range(total_pixels):
        if is_free_pixel(img, rr[i], cc[i], free_value_threshold):
            free_pixels += 1  # Incrementa se il pixel è libero

    # Calcola la proporzione di pixel liberi
    free_ratio = free_pixels / total_pixels

    # Se la proporzione di pixel liberi è maggiore o uguale alla tolleranza,
    # considera il segmento come sgombro (nessuna collisione)
    if free_ratio >= collision_tolerance:
        return False  # Nessuna collisione
    else:
        return True   # C'è collisione


def compute_distance(x1, y1, x2, y2):
    """
    Calcola la distanza euclidea tra due punti nel piano.

    Parametri:
        x1 (float): Coordinata x del primo punto.
        y1 (float): Coordinata y del primo punto.
        x2 (float): Coordinata x del secondo punto.
        y2 (float): Coordinata y del secondo punto.

    Ritorna:
        float: Distanza euclidea tra i due punti.
    """
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)  # Formula della distanza euclidea
    return distance  # Restituisce la distanza


###############################################################################
# Funzioni Modulari
###############################################################################

def build_initial_graph(nodes, map_info, img, h, k, max_edges_per_node, free_value_thresh, collision_tolerance, max_distance):
    """
    Costruisce il grafo iniziale basato sul K-Nearest Neighbors (K-NN),
    tenendo conto delle collisioni e della distanza massima.

    Parametri:
        nodes (list): Lista di nodi come tuple (label, x, y).
        map_info (dict): Dizionario contenente le informazioni della mappa.
        img (numpy.ndarray): Immagine della mappa in scala di grigi.
        h (int): Altezza dell'immagine in pixel.
        k (int): Numero di vicini da considerare per ogni nodo (K-NN).
        max_edges_per_node (int): Numero massimo di archi per nodo dopo il controllo collisioni.
        free_value_thresh (float): Soglia per considerare un pixel libero.
        collision_tolerance (float): Percentuale minima di pixel liberi per considerare un segmento sgombro.
        max_distance (float): Distanza euclidea massima ammessa per connettere due nodi.

    Ritorna:
        list: Lista di archi iniziali come dizionari {"source": ..., "target": ..., "distance": ...}.
    """
    # Estrae le coordinate dei nodi in un array NumPy per l'elaborazione
    coords = np.array([[nd[1], nd[2]] for nd in nodes], dtype=np.float32)  # Forma (N, 2)

    # Estrae le etichette dei nodi in una lista separata
    labels = [nd[0] for nd in nodes]  # Lista di etichette per riferimento

    if USE_SKLEARN:
        # Se sklearn è disponibile, utilizza KDTree per trovare i vicini più vicini
        tree = KDTree(coords)  # Costruisce l'albero KD
        # Esegue una query K-NN per ogni nodo, includendo se stesso (k+1)
        all_neighbors_idx = tree.query(coords, k=k+1, return_distance=True)
        all_distances = all_neighbors_idx[0]  # Matrice delle distanze
        all_indices = all_neighbors_idx[1]    # Matrice degli indici dei vicini
    else:
        # Se sklearn non è disponibile, utilizza un approccio all-pairs meno efficiente
        N = len(coords)  # Numero totale di nodi
        all_indices = []    # Lista per gli indici dei vicini
        all_distances = []  # Lista per le distanze dei vicini

        for i in range(N):
            dlist = []  # Lista temporanea per le distanze e gli indici dei vicini

            for j in range(N):
                if i == j:
                    continue  # Salta se è lo stesso nodo

                # Calcola la distanza euclidea tra il nodo i e il nodo j
                dist = compute_distance(coords[i][0], coords[i][1],
                                        coords[j][0], coords[j][1])
                dlist.append((dist, j))  # Aggiunge la tupla (distanza, indice)

            # Ordina la lista per distanza crescente
            dlist.sort(key=lambda x: x[0])

            # Prende i primi k vicini
            knn = dlist[:k]

            # Estrae solo gli indici e le distanze
            neighbor_indices = [x[1] for x in knn]
            neighbor_dists = [x[0] for x in knn]

            # Per coerenza con KDTree, aggiunge se stesso come primo vicino
            neighbor_indices = [i] + neighbor_indices
            neighbor_dists = [0.0] + neighbor_dists

            # Aggiunge gli indici e le distanze alle rispettive liste
            all_indices.append(neighbor_indices)
            all_distances.append(neighbor_dists)

        # Converte le liste in array NumPy per l'efficienza
        all_indices = np.array(all_indices, dtype=int)       # Forma (N, k+1)
        all_distances = np.array(all_distances, dtype=float)  # Forma (N, k+1)

    # Inizializza una lista di adiacenza per ogni nodo
    adjacency_list = [[] for _ in range(len(nodes))]

    # Itera su ogni nodo per determinare i suoi archi
    for i in range(len(nodes)):
        neighbor_idx_row = all_indices[i]      # Indici dei vicini per il nodo i
        neighbor_dist_row = all_distances[i]   # Distanze dei vicini per il nodo i

        candidate_list = []  # Lista temporanea per i candidati a essere archi

        # Itera su ogni vicino (salta il primo se è lo stesso nodo)
        for jj in range(1, len(neighbor_idx_row)):
            j = neighbor_idx_row[jj]          # Indice del vicino
            dist_ij = neighbor_dist_row[jj]   # Distanza dal nodo i al nodo j

            if j == i:
                continue  # Salta se è lo stesso nodo (già gestito)

            # Verifica se la distanza è entro il limite massimo
            if dist_ij > max_distance:
                # Salta questa connessione se la distanza supera il limite
                continue

            # Estrae le informazioni dei nodi i e j
            label_i, x_i, y_i = nodes[i]
            label_j, x_j, y_j = nodes[j]

            # Converte le coordinate mondo in pixel per entrambi i nodi
            r1, c1 = world_to_map(x_i, y_i, map_info, h)
            r2, c2 = world_to_map(x_j, y_j, map_info, h)

            # Verifica se c'è collisione lungo il percorso tra i due nodi
            has_collision = check_collision(
                img, r1, c1, r2, c2,
                free_value_threshold=free_value_thresh,
                collision_tolerance=collision_tolerance
            )

            # Se NON c'è collisione, aggiunge il vicino come candidato
            if not has_collision:
                candidate_list.append((j, dist_ij))  # Aggiunge la tupla (indice, distanza)

            # Se c'è collisione, non aggiunge l'arco

        # Ordina i candidati per distanza crescente
        candidate_list.sort(key=lambda x: x[1])

        # Tieni solo i primi 'max_edges_per_node' archi per questo nodo
        chosen = candidate_list[:max_edges_per_node]

        # Assegna gli archi scelti alla lista di adiacenza del nodo i
        adjacency_list[i] = chosen

    # Inizializza una lista per memorizzare gli archi finali
    edges_out = []

    # Costruisce la lista di archi basata sulla lista di adiacenza
    for i in range(len(nodes)):
        label_i = nodes[i][0]  # Etichetta del nodo i

        for (j, dist_ij) in adjacency_list[i]:
            label_j = nodes[j][0]  # Etichetta del nodo j

            # Crea un dizionario rappresentante l'arco
            edge_dict = {
                "source": label_i,      # Nodo di partenza
                "target": label_j,      # Nodo di arrivo
                "distance": dist_ij     # Distanza tra i nodi
            }

            # Aggiunge il dizionario alla lista degli archi
            edges_out.append(edge_dict)

    return edges_out  # Restituisce la lista degli archi iniziali


def connect_disconnected_components(G, nodes, map_info, img, h, collision_tolerance, free_value_thresh, max_distance, max_edges_per_node):
    """
    Verifica la connettività del grafo e connette le componenti disconnesse
    scegliendo le coppie di nodi con la minima distanza euclidea, preferendo nodi con meno archi,
    entro un limite massimo di distanza e di archi per nodo.

    Aggiorna la lista di archi e disegna gli archi aggiunti sull'immagine.

    Parametri:
        G (networkx.Graph): Grafo NetworkX esistente.
        nodes (list): Lista di nodi come tuple (label, x, y).
        map_info (dict): Dizionario contenente le informazioni della mappa.
        img (numpy.ndarray): Immagine della mappa in scala di grigi.
        h (int): Altezza dell'immagine in pixel.
        collision_tolerance (float): Percentuale minima di pixel liberi richiesta.
        free_value_thresh (float): Soglia per considerare un pixel libero.
        max_distance (float): Distanza euclidea massima ammessa per connettere due nodi.
        max_edges_per_node (int): Numero massimo di archi per nodo.

    Ritorna:
        list: Lista di archi aggiunti come dizionari {"source": ..., "target": ..., "distance": ...}.
    """
    # Controlla se il grafo è già completamente connesso
    if nx.is_connected(G):
        print("Il grafo è già connesso.")
        return []  # Nessun arco aggiunto

    print("Il grafo è disconnesso. Tentativo di connettere le componenti.")

    # Identifica tutte le componenti connesse nel grafo
    components = list(nx.connected_components(G))  # Lista di set, ogni set è una componente connessa
    edges_added = []  # Lista per memorizzare gli archi aggiunti

    # Ottiene il grado corrente di ogni nodo nel grafo
    degrees = dict(G.degree())  # Dizionario {nodo: grado}

    # Continua ad aggiungere archi finché il grafo non è completamente connesso
    while len(components) > 1:
        candidate_pairs = []  # Lista temporanea per le coppie candidate a connettere le componenti

        # Trova tutte le possibili coppie di componenti da connettere
        for i in range(len(components)):
            for j in range(i + 1, len(components)):
                component_a = components[i]  # Prima componente
                component_b = components[j]  # Seconda componente

                # Itera su tutti i nodi nella prima componente
                for node_a in component_a:
                    # Itera su tutti i nodi nella seconda componente
                    for node_b in component_b:
                        # Estrae i dati dei nodi (label, x, y)
                        node_a_data = next(nd for nd in nodes if nd[0] == node_a)
                        node_b_data = next(nd for nd in nodes if nd[0] == node_b)

                        # Calcola la distanza euclidea tra i due nodi
                        distance = compute_distance(
                            node_a_data[1], node_a_data[2],
                            node_b_data[1], node_b_data[2]
                        )

                        # Verifica se la distanza è entro il limite massimo
                        if distance > max_distance:
                            continue  # Salta questa coppia di nodi

                        # Verifica se l'aggiunta dell'arco supererebbe il limite di archi per nodo
                        if degrees.get(node_a, 0) >= max_edges_per_node or degrees.get(node_b, 0) >= max_edges_per_node:
                            continue  # Salta questa coppia di nodi

                        # Converte le coordinate del mondo in coordinate di mappa (pixel)
                        r1, c1 = world_to_map(node_a_data[1], node_a_data[2], map_info, h)
                        r2, c2 = world_to_map(node_b_data[1], node_b_data[2], map_info, h)

                        # Verifica se c'è collisione lungo il percorso tra i due nodi
                        has_collision = check_collision(
                            img, r1, c1, r2, c2,
                            free_value_threshold=free_value_thresh,
                            collision_tolerance=collision_tolerance
                        )

                        # Se NON c'è collisione, aggiunge la coppia alla lista dei candidati
                        if not has_collision:
                            # Ottiene i gradi correnti dei nodi
                            degree_a = degrees.get(node_a, 0)
                            degree_b = degrees.get(node_b, 0)

                            # Aggiunge una tupla con (distanza, somma dei gradi, nodo_a, nodo_b)
                            candidate_pairs.append((distance, degree_a + degree_b, node_a, node_b))

        # Se non ci sono coppie candidate, non è possibile connettere ulteriormente le componenti
        if not candidate_pairs:
            print("Non è stato possibile connettere tutte le componenti senza collisioni o entro la distanza massima.")
            break  # Esce dal ciclo per evitare loop infiniti

        # Ordina le coppie candidate: prima per distanza crescente, poi per somma dei gradi (meno archi preferiti)
        candidate_pairs.sort(key=lambda x: (x[0], x[1]))

        # Seleziona la migliore coppia (minima distanza, meno archi)
        best_pair = candidate_pairs[0]
        distance, degree_sum, node_a, node_b = best_pair

        # Aggiunge l'arco al grafo
        G.add_edge(node_a, node_b, distance=distance)

        # Crea un dizionario rappresentante l'arco aggiunto
        edge_dict = {
            "source": node_a,    # Nodo di partenza
            "target": node_b,    # Nodo di arrivo
            "distance": distance  # Distanza tra i nodi
        }
        edges_added.append(edge_dict)  # Aggiunge l'arco alla lista degli archi aggiunti

        # Aggiorna i gradi dei nodi coinvolti
        degrees[node_a] = degrees.get(node_a, 0) + 1
        degrees[node_b] = degrees.get(node_b, 0) + 1

        # Stampa un messaggio informativo
        print(f"Aggiungo arco tra '{node_a}' e '{node_b}' per connettere le componenti.")

        # Stampa condizionale per analizzare specificamente le connessioni tra node_19 e node_23
        if ((node_a == "node_19" and node_b == "node_23") or
            (node_a == "node_23" and node_b == "node_19")):
            print(f"Analisi specifica tra '{node_a}' e '{node_b}':")
            print(f"  Distanza Euclidea: {distance:.2f}")
            print(f"  Collisione: No")  # Poiché has_collision è False

        # Aggiorna la lista delle componenti connesse dopo l'aggiunta dell'arco
        components = list(nx.connected_components(G))

    # Dopo il tentativo di connettere le componenti, verifica la connettività finale del grafo
    if nx.is_connected(G):
        print("Il grafo è stato connesso.")
    else:
        print("Il grafo non è completamente connesso. Alcune componenti rimangono separate.")

    # Restituisce la lista degli archi aggiunti
    return edges_added


def draw_graph(img_color, nodes, edges, map_info, h):
    """
    Disegna gli archi del grafo sull'immagine colorata.

    Parametri:
        img_color (numpy.ndarray): Immagine della mappa in formato BGR (colorata).
        nodes (list): Lista di nodi come tuple (label, x, y).
        edges (list): Lista di archi come dizionari {"source": ..., "target": ..., "distance": ...}.
        map_info (dict): Dizionario contenente le informazioni della mappa.
        h (int): Altezza dell'immagine in pixel.
    """
    # Itera su ogni arco nella lista degli archi
    for edge in edges:
        label_i = edge["source"]  # Etichetta del nodo di partenza
        label_j = edge["target"]  # Etichetta del nodo di arrivo

        # Trova i dati dei nodi corrispondenti agli label
        node_i = next(nd for nd in nodes if nd[0] == label_i)
        node_j = next(nd for nd in nodes if nd[0] == label_j)

        # Converte le coordinate mondo in pixel per entrambi i nodi
        r1, c1 = world_to_map(node_i[1], node_i[2], map_info, h)
        r2, c2 = world_to_map(node_j[1], node_j[2], map_info, h)

        # Disegna una linea rossa tra i due nodi sull'immagine colorata
        # Parametri della funzione cv2.line:
        # - img_color: l'immagine su cui disegnare
        # - (c1, r1): coordinate del primo punto (colonna, riga)
        # - (c2, r2): coordinate del secondo punto (colonna, riga)
        # - (0, 0, 255): colore rosso in formato BGR
        # - 1: spessore della linea
        cv2.line(img_color, (c1, r1), (c2, r2), (0, 0, 255), 1)


def draw_nodes_and_labels(img_color, nodes, map_info, h):
    """
    Disegna i nodi e le loro etichette sull'immagine colorata.

    Parametri:
        img_color (numpy.ndarray): Immagine della mappa in formato BGR (colorata).
        nodes (list): Lista di nodi come tuple (label, x, y).
        map_info (dict): Dizionario contenente le informazioni della mappa.
        h (int): Altezza dell'immagine in pixel.
    """
    # Itera su ogni nodo nella lista dei nodi
    for node in nodes:
        label, x, y = node  # Estrae etichetta e coordinate del nodo

        # Converte le coordinate mondo in pixel
        r, c = world_to_map(x, y, map_info, h)

        # Disegna un cerchio verde al punto (c, r) sull'immagine colorata
        # Parametri della funzione cv2.circle:
        # - img_color: l'immagine su cui disegnare
        # - (c, r): coordinate del centro del cerchio (colonna, riga)
        # - radius=5: raggio del cerchio in pixel
        # - color=(0, 255, 0): colore verde in formato BGR
        # - thickness=-1: riempimento del cerchio
        cv2.circle(img_color, (c, r), radius=5, color=(0, 255, 0), thickness=-1)

        # Aggiunge l'etichetta del nodo vicino al cerchio
        # Imposta le proprietà del testo
        font = cv2.FONT_HERSHEY_SIMPLEX  # Tipo di font
        font_scale = 0.4                  # Scala del font
        font_thickness = 1                # Spessore del testo

        # Calcola la dimensione del testo per posizionarlo correttamente
        text_size, _ = cv2.getTextSize(label, font, font_scale, font_thickness)

        # Calcola le coordinate del testo con un piccolo offset per evitare sovrapposizioni
        text_x = c + 6  # Offset orizzontale
        text_y = r - 6  # Offset verticale

        # Scrive l'etichetta sul'immagine
        # Parametri della funzione cv2.putText:
        # - img_color: l'immagine su cui disegnare
        # - label: testo da disegnare
        # - (text_x, text_y): coordinate del punto di inizio del testo
        # - font: tipo di font
        # - font_scale: scala del font
        # - (255, 0, 0): colore blu in formato BGR
        # - font_thickness: spessore del testo
        # - cv2.LINE_AA: tipo di linea (antialiased)
        cv2.putText(img_color, label, (text_x, text_y), font, font_scale, (255, 0, 0), font_thickness, cv2.LINE_AA)


def save_graph_json(nodes, edges, output_json):
    """
    Salva il grafo in un file JSON con una struttura specifica.

    Struttura JSON:
    {
      "nodes": [
        {
          "label": "node_1",
          "x": ...,
          "y": ...
        },
        ...
      ],
      "edges": [
        {
          "source": "node_1",
          "target": "node_2",
          "distance": ...
        },
        ...
      ]
    }

    Parametri:
        nodes (list): Lista di nodi come tuple (label, x, y).
        edges (list): Lista di archi come dizionari {"source": ..., "target": ..., "distance": ...}.
        output_json (str): Percorso al file JSON di output.
    """
    # Crea un dizionario per il grafo risultante
    result_graph = {
        "nodes": [
            {
                "label": nd[0],  # Etichetta del nodo
                "x": nd[1],      # Coordinata x nel mondo
                "y": nd[2]       # Coordinata y nel mondo
            } for nd in nodes
        ],
        "edges": [
            {
                "source": edge["source"],  # Nodo di partenza
                "target": edge["target"],  # Nodo di arrivo
                "distance": edge["distance"]  # Distanza tra i nodi
            } for edge in edges
        ]
    }

    # Apre il file JSON di output in modalità scrittura
    with open(output_json, 'w') as f:
        json.dump(result_graph, f, indent=2)  # Scrive il dizionario in formato JSON con indentazione

    # Stampa un messaggio di conferma
    print(f"Grafo costruito correttamente e salvato in {output_json}.")


def save_graph_image(img_color, output_image):
    """
    Salva l'immagine con il grafo disegnato.

    Parametri:
        img_color (numpy.ndarray): Immagine della mappa con il grafo disegnato (in formato BGR).
        output_image (str): Percorso al file immagine di output.
    """
    # Salva l'immagine usando OpenCV
    cv2.imwrite(output_image, img_color)

    # Stampa un messaggio di conferma
    print(f"Immagine con il grafo disegnato salvata in {output_image}.")


###############################################################################
# Funzione principale
###############################################################################

def main():
    """
    Funzione principale che coordina tutte le operazioni per costruire il grafo
    di navigazione, gestire le collisioni, connettere componenti disconnesse e
    salvare i risultati in file JSON e immagine.
    """
    # Crea un parser per gestire gli argomenti da riga di comando
    parser = argparse.ArgumentParser(
        description="Costruisce un grafo di navigazione con tolleranza alle collisioni e salva l'immagine con il grafo disegnato."
    )

    # Aggiunge gli argomenti necessari al parser
    parser.add_argument("--map_yaml", type=str, required=True,
                        help="Percorso al file .yaml della mappa (Occupancy Grid).")
    parser.add_argument("--graph_json", type=str, required=True,
                        help="Percorso al file .json contenente i nodi (waypoints).")
    parser.add_argument("--k", type=int, default=6,
                        help="Numero di vicini da considerare per ogni nodo (K-NN). Deve essere >= max_edges_per_node.")
    parser.add_argument("--max_edges_per_node", type=int, default=4,
                        help="Numero massimo di archi da mantenere per ogni nodo dopo il controllo collisioni.")
    parser.add_argument("--output_json", type=str, default="output_graph.json",
                        help="Nome del file JSON di output con il grafo completo.")
    parser.add_argument("--output_image", type=str, default="graph_on_map.png",
                        help="Nome del file immagine di output con il grafo disegnato.")

    # Soglia per considerare un pixel libero (es. >=110 se 111=bianco)
    parser.add_argument("--free_value_thresh", type=float, default=110.0,
                        help="Valore di soglia per considerare un pixel libero. Esempio: 110 (se 111=bianco).")

    # Tolleranza collisione: es. 0.8 => se l'80% dei pixel è libero, niente collisione
    parser.add_argument("--collision_tolerance", type=float, default=1.0,
                        help="Percentuale [0..1] di pixel liberi richiesta per considerare un segmento sgombro (no collision).")

    # Nuovo parametro: distanza massima consentita tra due nodi per connetterli
    parser.add_argument("--max_distance", type=float, required=True,
                        help="Distanza euclidea massima ammessa tra due nodi per connetterli tramite un arco.")

    # Parsing degli argomenti forniti dall'utente
    args = parser.parse_args()

    # Verifica che il numero di vicini 'k' sia almeno pari a 'max_edges_per_node'
    if args.k < args.max_edges_per_node:
        print(f"Errore: il parametro --k ({args.k}) deve essere >= --max_edges_per_node ({args.max_edges_per_node}).")
        return  # Esce dalla funzione principale

    # 1) Carica le informazioni della mappa dal file YAML
    map_info = load_map_info(args.map_yaml)

    # 2) Carica l'immagine della mappa in scala di grigi
    img, h, w = load_occupancy_image(map_info)

    # Converti l'immagine in formato BGR per permettere disegni a colori
    img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # 3) Carica i nodi (waypoints) dal file JSON
    nodes = load_nodes(args.graph_json)
    # nodes è una lista di tuple: [(label, x, y), ...]

    # 4) Costruisci il grafo iniziale basato su K-NN, collisioni e distanza
    initial_edges = build_initial_graph(
        nodes=nodes,
        map_info=map_info,
        img=img,
        h=h,
        k=args.k,
        max_edges_per_node=args.max_edges_per_node,
        free_value_thresh=args.free_value_thresh,
        collision_tolerance=args.collision_tolerance,
        max_distance=args.max_distance  # Passa il parametro della distanza massima
    )

    # 5) Disegna gli archi iniziali sull'immagine colorata
    draw_graph(img_color, nodes, initial_edges, map_info, h)

    # 6) Crea un grafo NetworkX non orientato
    G = nx.Graph()  # Inizializza un grafo non orientato
    G.add_nodes_from([node[0] for node in nodes])  # Aggiunge tutti i nodi al grafo
    G.add_edges_from([(edge["source"], edge["target"]) for edge in initial_edges])  # Aggiunge gli archi iniziali

    # 7) Verifica e connette eventuali componenti disconnesse nel grafo
    edges_added = connect_disconnected_components(
        G=G,
        nodes=nodes,
        map_info=map_info,
        img=img,
        h=h,
        collision_tolerance=args.collision_tolerance,
        free_value_thresh=args.free_value_thresh,
        max_distance=args.max_distance,          # Passa il parametro della distanza massima
        max_edges_per_node=args.max_edges_per_node  # Passa il parametro del limite degli archi
    )

    # 8) Disegna gli archi aggiunti sulla mappa colorata
    for edge in edges_added:
        source, target, distance = edge["source"], edge["target"], edge["distance"]  # Estrae i dati dell'arco
        node_source = next(nd for nd in nodes if nd[0] == source)  # Trova il nodo di partenza
        node_target = next(nd for nd in nodes if nd[0] == target)  # Trova il nodo di arrivo

        # Converte le coordinate mondo in pixel per entrambi i nodi
        r1, c1 = world_to_map(node_source[1], node_source[2], map_info, h)
        r2, c2 = world_to_map(node_target[1], node_target[2], map_info, h)

        # Disegna una linea rossa tra i nodi aggiunti
        cv2.line(img_color, (c1, r1), (c2, r2), (0, 0, 255), 1)  # Colore rosso (BGR: 0,0,255)

    # 9) Disegna i nodi e le etichette sull'immagine colorata
    draw_nodes_and_labels(img_color, nodes, map_info, h)

    # 10) Prepara la lista finale di archi combinando archi iniziali e aggiunti
    final_edges = initial_edges + edges_added

    # -- Rimuoviamo eventuali duplicati: (source, target) e (target, source) devono essere lo stesso arco --
    unique_edges_set = set()      # Insieme per tracciare gli archi unici
    unique_edges_list = []        # Lista finale senza duplicati

    for e in final_edges:
        s = e["source"]            # Nodo di partenza
        t = e["target"]            # Nodo di arrivo
        d = e["distance"]          # Distanza tra i nodi

        # Normalizza l'ordine delle etichette per garantire unicità (min, max)
        if s > t:
            s, t = t, s  # Scambia s e t se necessario

        # Crea una tupla per identificare l'arco univocamente
        edge_tuple = (s, t, d)

        # Se l'arco non è già presente nell'insieme, aggiungilo
        if edge_tuple not in unique_edges_set:
            unique_edges_set.add(edge_tuple)  # Aggiunge la tupla all'insieme

            # Crea un dizionario rappresentante l'arco senza duplicati
            edge_dict = {
                "source": s,       # Nodo di partenza
                "target": t,       # Nodo di arrivo
                "distance": d      # Distanza tra i nodi
            }
            unique_edges_list.append(edge_dict)  # Aggiunge il dizionario alla lista finale

    # Assegna la lista senza duplicati a final_edges
    final_edges = unique_edges_list

    # 11) Salva il grafo finale in un file JSON
    save_graph_json(nodes, final_edges, args.output_json)

    # 12) Salva l'immagine con il grafo disegnato
    save_graph_image(img_color, args.output_image)


###############################################################################
# Entry point
###############################################################################

if __name__ == "__main__":
    main()  # Esegue la funzione principale quando lo script viene eseguito direttamente
