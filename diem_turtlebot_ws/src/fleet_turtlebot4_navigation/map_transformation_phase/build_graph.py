#!/usr/bin/env python3

import argparse
import json
import cv2
import numpy as np
import yaml
import math
import os

# Per la linea Bresenham
from skimage.draw import line

try:
    from sklearn.neighbors import KDTree
    USE_SKLEARN = True
except ImportError:
    # Se non hai sklearn, metti a False e userai un all-pairs distance
    USE_SKLEARN = False

# Importa NetworkX per la gestione della connettività
import networkx as nx

###############################################################################
# Funzioni di supporto
###############################################################################

def load_map_info(yaml_file):
    """
    Carica le informazioni di mappa da file .yaml
    """
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)
    map_info = {}
    yaml_dir = os.path.dirname(os.path.abspath(yaml_file))
    image_rel_path = data["image"]  # es: './diem_map.pgm'
    image_path = os.path.join(yaml_dir, image_rel_path)

    map_info["image_path"] = image_path
    map_info["resolution"] = float(data["resolution"])
    map_info["origin"] = data["origin"]  # [origin_x, origin_y, origin_theta]
    map_info["negate"] = int(data.get("negate", 0))
    map_info["occupied_thresh"] = float(data.get("occupied_thresh", 0.65))
    map_info["free_thresh"] = float(data.get("free_thresh", 0.196))

    return map_info

def load_occupancy_image(map_info):
    """
    Carica la mappa in scala di grigi con OpenCV.
    """
    img = cv2.imread(map_info["image_path"], cv2.IMREAD_UNCHANGED)
    if img is None:
        raise FileNotFoundError(f"Impossibile caricare l'immagine: {map_info['image_path']}")

    # Se necessario invertire i valori in base a 'negate'
    if map_info["negate"] == 1:
        img = 255 - img

    height, width = img.shape
    return img, height, width

def load_nodes(json_file):
    """
    Carica i nodi (waypoints) da un file JSON con struttura:
    {
      "nodes": [
        { "label": "node_1", "x": ..., "y": ... },
        ...
      ]
    }
    Restituisce una lista di tuple (label, x, y).
    """
    with open(json_file, 'r') as f:
        data = json.load(f)
    nodes_data = data["nodes"]
    nodes_list = []
    for nd in nodes_data:
        label = nd["label"]
        x = nd["x"]
        y = nd["y"]
        nodes_list.append((label, x, y))
    return nodes_list

def world_to_map(x_world, y_world, map_info, map_height):
    """
    Converte coordinate (x_world, y_world) in pixel (riga, colonna) dell'immagine
    assumendo che:
      - origin_x, origin_y sia in basso a sinistra
      - risoluzione in metri/pixel
      - l'immagine in OpenCV abbia (0,0) in alto a sinistra

    col = int( (x_world - origin_x) / resolution )
    row = int( map_height - 1 - (y_world - origin_y) / resolution )
    """
    origin_x, origin_y, _ = map_info["origin"]
    resolution = map_info["resolution"]

    col_f = (x_world - origin_x) / resolution
    row_f = (y_world - origin_y) / resolution

    col = int(math.floor(col_f))
    row = int(math.floor(map_height - 1 - row_f))

    return row, col

def is_free_pixel(img, row, col, free_value_threshold):
    """
    Verifica se il pixel in (row, col) è considerato 'free'.
    """
    h, w = img.shape
    if row < 0 or row >= h or col < 0 or col >= w:
        return False  # Fuori dai limiti => consideralo occupato
    return (img[row, col] >= free_value_threshold)

def check_collision(img, r1, c1, r2, c2, free_value_threshold=50, collision_tolerance=1.0):
    """
    Verifica se il segmento (r1,c1)->(r2,c2) attraversa pixel non liberi,
    tenendo conto di una soglia di tolleranza (collision_tolerance).

    Se la percentuale di pixel liberi lungo la linea Bresenham è
    >= collision_tolerance, allora consideriamo "nessuna collisione".
    Altrimenti "collisione".

    Ritorna True se c'è collisione, False se NON c'è collisione.
    """
    rr, cc = line(r1, c1, r2, c2)
    total_pixels = len(rr)
    free_pixels = 0

    for i in range(total_pixels):
        if is_free_pixel(img, rr[i], cc[i], free_value_threshold):
            free_pixels += 1

    free_ratio = free_pixels / total_pixels

    # Se la frazione di pixel liberi è >= collision_tolerance => NO collisione
    if free_ratio >= collision_tolerance:
        return False  # no collision
    else:
        return True   # collision

def compute_distance(x1, y1, x2, y2):
    """
    Calcola la distanza euclidea tra due punti nel piano.
    """
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distance

###############################################################################
# Funzioni Modulari
###############################################################################

def build_initial_graph(nodes, map_info, img, h, k, max_edges_per_node, free_value_thresh, collision_tolerance, max_distance):
    """
    Costruisce il grafo iniziale basato su K-NN, controllo delle collisioni e distanza massima.
    Restituisce la lista di archi (edges_out) come dizionari.

    Parametri aggiunti:
    - max_distance: Distanza euclidea massima ammessa per connettere due nodi.
    """
    coords = np.array([[nd[1], nd[2]] for nd in nodes], dtype=np.float32)  # (N, 2)
    labels = [nd[0] for nd in nodes]  # Per riferimento

    if USE_SKLEARN:
        tree = KDTree(coords)
        # Indici e distanze dei k vicini + se stesso (k+1)
        all_neighbors_idx = tree.query(coords, k=k+1, return_distance=True)
        all_distances = all_neighbors_idx[0]
        all_indices = all_neighbors_idx[1]
    else:
        # All-pairs approach (meno efficiente ma semplice)
        N = len(coords)
        all_indices = []
        all_distances = []
        for i in range(N):
            dlist = []
            for j in range(N):
                if i == j:
                    continue
                dist = compute_distance(coords[i][0], coords[i][1],
                                        coords[j][0], coords[j][1])
                dlist.append((dist, j))
            # Ordina per distanza
            dlist.sort(key=lambda x: x[0])
            # Prendi i primi k
            knn = dlist[:k]
            # Salva i soli indici e distanze
            neighbor_indices = [x[1] for x in knn]
            neighbor_dists = [x[0] for x in knn]

            # Per coerenza con KDTree, aggiungiamo "self" come primo?
            neighbor_indices = [i] + neighbor_indices
            neighbor_dists = [0.0] + neighbor_dists

            all_indices.append(neighbor_indices)
            all_distances.append(neighbor_dists)

        all_indices = np.array(all_indices, dtype=int)     # (N, k+1)
        all_distances = np.array(all_distances, dtype=float)  # (N, k+1)

    # Costruzione della lista di adiacenza
    adjacency_list = [[] for _ in range(len(nodes))]

    for i in range(len(nodes)):
        neighbor_idx_row = all_indices[i]
        neighbor_dist_row = all_distances[i]

        candidate_list = []
        for jj in range(1, len(neighbor_idx_row)):
            j = neighbor_idx_row[jj]
            dist_ij = neighbor_dist_row[jj]
            if j == i:
                continue

            # Verifica se la distanza è entro il limite massimo
            if dist_ij > max_distance:
                # print(f"Salto connessione tra '{nodes[i][0]}' e '{nodes[j][0]}' perché la distanza {dist_ij:.2f} supera il massimo consentito {max_distance}.")
                continue

            # Conversione in pixel
            label_i, x_i, y_i = nodes[i]
            label_j, x_j, y_j = nodes[j]

            r1, c1 = world_to_map(x_i, y_i, map_info, h)
            r2, c2 = world_to_map(x_j, y_j, map_info, h)

            # Check collision con tolleranza
            has_collision = check_collision(
                img, r1, c1, r2, c2,
                free_value_threshold=free_value_thresh,
                collision_tolerance=collision_tolerance
            )
            # Se NON c'è collisione => aggiungiamo come arco
            if not has_collision:
                candidate_list.append((j, dist_ij))
            # else:
                # print(f"Connessione tra '{nodes[i][0]}' e '{nodes[j][0]}' saltata a causa di collisioni.")

        # Teniamo solo i primi 'max_edges_per_node' per distanza
        candidate_list.sort(key=lambda x: x[1])
        chosen = candidate_list[:max_edges_per_node]
        adjacency_list[i] = chosen

    # Costruzione della lista di archi
    edges_out = []
    for i in range(len(nodes)):
        label_i = nodes[i][0]
        for (j, dist_ij) in adjacency_list[i]:
            label_j = nodes[j][0]
            edge_dict = {
                "source": label_i,
                "target": label_j,
                "distance": dist_ij
            }
            edges_out.append(edge_dict)

    return edges_out


def connect_disconnected_components(G, nodes, map_info, img, h, collision_tolerance, free_value_thresh, max_distance, max_edges_per_node):
    """
    Verifica la connettività del grafo e connette le componenti disconnesse
    scegliendo le coppie di nodi con la minima distanza euclidea, preferendo nodi con meno archi,
    entro un limite massimo di distanza e di archi per nodo.
    Aggiorna la lista di archi e disegna gli archi aggiunti sull'immagine.
    Restituisce la lista aggiunta di archi come dizionari.

    Parametri:
    - G: Grafo NetworkX esistente.
    - nodes: Lista di nodi, dove ogni nodo è una tupla (label, x, y).
    - map_info: Dizionario contenente le informazioni della mappa.
    - img: Immagine della mappa in scala di grigi.
    - h: Altezza dell'immagine.
    - collision_tolerance: Tolleranza per la collisione (percentuale di pixel liberi richiesta).
    - free_value_thresh: Soglia per considerare un pixel come libero.
    - max_distance: Distanza euclidea massima ammessa per connettere due nodi.
    - max_edges_per_node: Numero massimo di archi per nodo.

    Ritorna:
    - edges_added: Lista di archi aggiunti come dizionari {"source": ..., "target": ..., "distance": ...}.
    """
    # Controlla se il grafo è già connesso
    if nx.is_connected(G):
        print("Il grafo è già connesso.")
        return []  # Nessun arco aggiunto

    print("Il grafo è disconnesso. Tentativo di connettere le componenti.")
    
    # Identifica tutte le componenti connesse nel grafo
    components = list(nx.connected_components(G))
    edges_added = []  # Lista per memorizzare gli archi aggiunti

    # Ottieni il grado corrente di ogni nodo
    degrees = dict(G.degree())

    # Continua ad aggiungere archi finché il grafo non è completamente connesso
    while len(components) > 1:
        candidate_pairs = []

        # Trova tutte le possibili coppie di componenti
        for i in range(len(components)):
            for j in range(i + 1, len(components)):
                component_a = components[i]
                component_b = components[j]

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
                        
                        if not has_collision:
                            # Ottieni i gradi correnti dei nodi
                            degree_a = degrees.get(node_a, 0)
                            degree_b = degrees.get(node_b, 0)
                            candidate_pairs.append((distance, degree_a + degree_b, node_a, node_b))
    
        if not candidate_pairs:
            print("Non è stato possibile connettere tutte le componenti senza collisioni o entro la distanza massima.")
            break  # Esce dal ciclo per evitare loop infiniti

        # Ordina le coppie: prima per distanza, poi per somma dei gradi (meno archi preferiti)
        candidate_pairs.sort(key=lambda x: (x[0], x[1]))
        
        # Seleziona la migliore coppia
        best_pair = candidate_pairs[0]
        distance, degree_sum, node_a, node_b = best_pair

        # Aggiungi l'arco al grafo
        G.add_edge(node_a, node_b, distance=distance)
        
        # Aggiorna la lista degli archi aggiunti
        edge_dict = {
            "source": node_a,
            "target": node_b,
            "distance": distance
        }
        edges_added.append(edge_dict)
        
        # Aggiorna i gradi dei nodi
        degrees[node_a] = degrees.get(node_a, 0) + 1
        degrees[node_b] = degrees.get(node_b, 0) + 1
        
        # Stampa un messaggio informativo
        print(f"Aggiungo arco tra '{node_a}' e '{node_b}' per connettere le componenti.")
        
        # Stampa condizionale per node_19 e node_23
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

    # Ritorna la lista degli archi aggiunti
    return edges_added


def draw_graph(img_color, nodes, edges, map_info, h):
    """
    Disegna gli archi sul grafico.
    """
    for edge in edges:
        label_i = edge["source"]
        label_j = edge["target"]

        node_i = next(nd for nd in nodes if nd[0] == label_i)
        node_j = next(nd for nd in nodes if nd[0] == label_j)

        r1, c1 = world_to_map(node_i[1], node_i[2], map_info, h)
        r2, c2 = world_to_map(node_j[1], node_j[2], map_info, h)

        # Disegna una linea di colore rosso (BGR: (0, 0, 255)), spessore 1
        cv2.line(img_color, (c1, r1), (c2, r2), (0, 0, 255), 1)

def draw_nodes_and_labels(img_color, nodes, map_info, h):
    """
    Disegna i nodi e le etichette sull'immagine.
    """
    for node in nodes:
        label, x, y = node
        r, c = world_to_map(x, y, map_info, h)
        
        # Disegna un cerchio verde (BGR: (0, 255, 0)), spessore -1 (pieno)
        cv2.circle(img_color, (c, r), radius=5, color=(0, 255, 0), thickness=-1)

        # Aggiungi l'etichetta (testo) vicino al nodo
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.4
        font_thickness = 1
        text_size, _ = cv2.getTextSize(label, font, font_scale, font_thickness)
        text_x = c + 6  # Offset orizzontale per non sovrapporre il testo al cerchio
        text_y = r - 6  # Offset verticale
        cv2.putText(img_color, label, (text_x, text_y), font, font_scale, (255, 0, 0), font_thickness, cv2.LINE_AA)

def save_graph_json(nodes, edges, output_json):
    """
    Salva il grafo in un file JSON.
    """
    result_graph = {
        "nodes": [
            {
                "label": nd[0],
                "x": nd[1],
                "y": nd[2]
            } for nd in nodes
        ],
        "edges": [
            {
                "source": edge["source"],
                "target": edge["target"],
                "distance": edge["distance"]
            } for edge in edges
        ]
    }

    with open(output_json, 'w') as f:
        json.dump(result_graph, f, indent=2)

    print(f"Grafo costruito correttamente e salvato in {output_json}.")

def save_graph_image(img_color, output_image):
    """
    Salva l'immagine con il grafo disegnato.
    """
    cv2.imwrite(output_image, img_color)
    print(f"Immagine con il grafo disegnato salvata in {output_image}.")

###############################################################################
# Funzione principale
###############################################################################

def main():
    parser = argparse.ArgumentParser(description="Costruisce un grafo di navigazione con tolleranza alle collisioni e salva l'immagine con il grafo disegnato.")
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

    args = parser.parse_args()

    # Verifica che k sia almeno pari a max_edges_per_node
    if args.k < args.max_edges_per_node:
        print(f"Errore: il parametro --k ({args.k}) deve essere >= --max_edges_per_node ({args.max_edges_per_node}).")
        return

    # 1) Carica le informazioni della mappa
    map_info = load_map_info(args.map_yaml)
    img, h, w = load_occupancy_image(map_info)

    # Converti l'immagine in BGR per poter disegnare in colori
    img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # 2) Carica i nodi da file JSON
    nodes = load_nodes(args.graph_json)
    # nodes = [(label, x, y), ...]

    # 3) Costruisci il grafo iniziale
    initial_edges = build_initial_graph(
        nodes=nodes,
        map_info=map_info,
        img=img,
        h=h,
        k=args.k,
        max_edges_per_node=args.max_edges_per_node,
        free_value_thresh=args.free_value_thresh,
        collision_tolerance=args.collision_tolerance,
        max_distance=args.max_distance  # Passa il nuovo parametro
    )

    # 4) Disegna gli archi iniziali sull'immagine
    draw_graph(img_color, nodes, initial_edges, map_info, h)

    # 5) Crea il grafo NetworkX
    G = nx.Graph()
    G.add_nodes_from([node[0] for node in nodes])
    G.add_edges_from([(edge["source"], edge["target"]) for edge in initial_edges])

    # 6) Verifica e connette le componenti disconnesse
    edges_added = connect_disconnected_components(
        G=G,
        nodes=nodes,
        map_info=map_info,
        img=img,
        h=h,
        collision_tolerance=args.collision_tolerance,
        free_value_thresh=args.free_value_thresh,
        max_distance=args.max_distance,          # Passa il nuovo parametro
        max_edges_per_node=args.max_edges_per_node  # Passa il parametro di limite degli archi
    )

    # 7) Disegna gli archi aggiunti sulla mappa
    for edge in edges_added:
        source, target, distance = edge["source"], edge["target"], edge["distance"]
        node_source = next(nd for nd in nodes if nd[0] == source)
        node_target = next(nd for nd in nodes if nd[0] == target)

        r1, c1 = world_to_map(node_source[1], node_source[2], map_info, h)
        r2, c2 = world_to_map(node_target[1], node_target[2], map_info, h)

        cv2.line(img_color, (c1, r1), (c2, r2), (0, 0, 255), 1)

    # 8) Disegna i nodi e le etichette sull'immagine
    draw_nodes_and_labels(img_color, nodes, map_info, h)

    # 9) Prepara la lista finale di archi
    # initial_edges e edges_added sono entrambi list di dizionari
    # 9) Prepara la lista finale di archi
    final_edges = initial_edges + edges_added

    # -- Rimuoviamo eventuali duplicati: (source, target) e (target, source) devono essere lo stesso arco --
    unique_edges_set = set()
    unique_edges_list = []
    for e in final_edges:
        s = e["source"]
        t = e["target"]
        d = e["distance"]
        # Normalizza l'ordine (min, max)
        if s > t:
            s, t = t, s
        # Aggiungiamo al set la tupla (s, t, d)
        if (s, t, d) not in unique_edges_set:
            unique_edges_set.add((s, t, d))
            # Ricostruisci il dizionario con source < target
            edge_dict = {
                "source": s,
                "target": t,
                "distance": d
            }
            unique_edges_list.append(edge_dict)

    # Ora unique_edges_list contiene solo archi non duplicati
    final_edges = unique_edges_list


    # 10) Salva il grafo in JSON
    save_graph_json(nodes, final_edges, args.output_json)

    # 11) Salva l'immagine con il grafo disegnato
    save_graph_image(img_color, args.output_image)

###############################################################################
# Entry point
###############################################################################

if __name__ == "__main__":
    main()
