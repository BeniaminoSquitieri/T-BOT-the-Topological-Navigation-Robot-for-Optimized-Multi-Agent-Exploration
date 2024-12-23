#!/usr/bin/env python3

import argparse
import json
import os
import matplotlib.pyplot as plt
import networkx as nx
import sys
from PIL import Image
import numpy as np
import yaml

def load_map_info(yaml_file):
    """
    Carica le informazioni di mappa da file .yaml
    Ritorna un dizionario con:
      - 'image_path': path all'immagine .pgm (assoluto)
      - 'resolution': risoluzione (m/pixel)
      - 'origin': (origin_x, origin_y)  (ignoreremo l'angolo origin_theta)
    """
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)
    # Costruisci il path assoluto dell'immagine se serve
    yaml_dir = os.path.dirname(os.path.abspath(yaml_file))
    image_rel_path = data["image"]
    image_path = os.path.join(yaml_dir, image_rel_path)

    # Estrapola i valori dal dizionario YAML
    map_info = {
        "image_path": image_path,
        "resolution": float(data["resolution"]),
        # estrai i primi due elementi dell'origin [x, y]
        "origin": (float(data["origin"][0]), float(data["origin"][1]))
    }
    return map_info

def map_to_pixel(x_map, y_map, origin, resolution, image_height):
    """
    Converte le coordinate (x_map, y_map) (mondo RVIZ)
    in coordinate pixel (col, row) per l'immagine della mappa.
    
    Parametri:
        x_map, y_map (float): coordinate nel sistema mappa (metri).
        origin (tuple): (origin_x, origin_y) della mappa.
        resolution (float): risoluzione [m/pixel].
        image_height (int): altezza dell'immagine (in pixel).
        
    Ritorna:
        (int, int): coordinate pixel (x_pixel, y_pixel).
    """
    # Calcolo x_pixel
    x_pixel = int((x_map - origin[0]) / resolution)
    # Calcolo y_pixel, notare l'inversione (y cresce verso l'alto, in immagine row=0 è in alto)
    y_pixel = image_height - int((y_map - origin[1]) / resolution)
    return x_pixel, y_pixel

def main():
    parser = argparse.ArgumentParser(description="Visualizza un grafo su una mappa, leggendo i dati da YAML e JSON.")
    parser.add_argument("--map_yaml", required=True, help="Percorso al file .yaml della mappa (contenente image, resolution, origin, ecc.).")
    parser.add_argument("--graph_json", required=True, help="Percorso al file .json che descrive i nodi e gli archi del grafo.")
    # Opzionale: Percorso a un file JSON con archi aggiuntivi
    parser.add_argument("--additional_edges_json", required=False, help="Percorso al file .json contenente archi aggiuntivi da inserire nel grafo.")
    args = parser.parse_args()

    # 1) Carica info mappa dallo YAML
    map_info = load_map_info(args.map_yaml)
    image_path = map_info["image_path"]
    resolution = map_info["resolution"]
    origin = map_info["origin"]

    # 2) Carica immagine mappa
    if not os.path.exists(image_path):
        print(f"Errore: Impossibile trovare l'immagine della mappa: {image_path}")
        sys.exit(1)

    image = Image.open(image_path).convert("L")
    image_array = np.array(image)
    image_height, image_width = image_array.shape

    # 3) Carica grafo dal file JSON
    if not os.path.exists(args.graph_json):
        print(f"Errore: Impossibile trovare il file JSON del grafo: {args.graph_json}")
        sys.exit(1)

    with open(args.graph_json, 'r') as file:
        graph_data = json.load(file)

    # 4) Converti i nodi dalle coordinate mondo->pixel
    node_positions_pixel = {}
    for node in graph_data["nodes"]:
        x_map = node["x"]
        y_map = node["y"]
        label = node["label"]
        x_pixel, y_pixel = map_to_pixel(x_map, y_map, origin, resolution, image_height)
        node_positions_pixel[label] = (x_pixel, y_pixel)

    # 5) Estrai gli archi (gestisci "source" e "target")
    edges = []
    for edge in graph_data["edges"]:
        # Controlla se usa "source" e "target"
        if "source" in edge and "target" in edge:
            edges.append((edge["source"], edge["target"]))
        elif "from" in edge and "to" in edge:
            edges.append((edge["from"], edge["to"]))
        else:
            print(f"Attenzione: Arco con formato sconosciuto: {edge}")
            continue

    # 6) Crea il grafo NetworkX e aggiungi nodi ed archi
    G = nx.Graph()
    G.add_nodes_from(node_positions_pixel.keys())
    G.add_edges_from(edges)

    # 7) Aggiungi archi aggiuntivi se forniti
    added_edges = []
    if args.additional_edges_json:
        if not os.path.exists(args.additional_edges_json):
            print(f"Errore: Impossibile trovare il file JSON degli archi aggiuntivi: {args.additional_edges_json}")
            sys.exit(1)
        with open(args.additional_edges_json, 'r') as file:
            additional_data = json.load(file)
        for edge in additional_data["edges"]:
            if "source" in edge and "target" in edge:
                source = edge["source"]
                target = edge["target"]
                # Verifica che i nodi esistano
                if source in node_positions_pixel and target in node_positions_pixel:
                    G.add_edge(source, target)
                    added_edges.append((source, target))
                else:
                    print(f"Attenzione: Nodi non trovati per aggiungere l'arco: {source} - {target}")
            elif "from" in edge and "to" in edge:
                source = edge["from"]
                target = edge["to"]
                # Verifica che i nodi esistano
                if source in node_positions_pixel and target in node_positions_pixel:
                    G.add_edge(source, target)
                    added_edges.append((source, target))
                else:
                    print(f"Attenzione: Nodi non trovati per aggiungere l'arco: {source} - {target}")
            else:
                print(f"Attenzione: Arco aggiuntivo con formato sconosciuto: {edge}")
                continue

    # 8) Prepara directory di output
    output_directory = "graph"
    os.makedirs(output_directory, exist_ok=True)
    # Nome del file in base al JSON
    json_file_name = os.path.splitext(os.path.basename(args.graph_json))[0]
    output_png_path = os.path.join(output_directory, f"{json_file_name}_graph_map.png")

    # 9) Prepara posizioni per la visualizzazione
    pos = node_positions_pixel

    # 10) Identifica gli archi originali e aggiunti
    original_edges = set(edges)
    added_edges_set = set(added_edges)

    # 11) Crea due liste separate di archi
    original_edges_to_draw = list(original_edges)
    added_edges_to_draw = list(added_edges_set)

    # 12) Visualizza/plotta la mappa e il grafo sovrapposto
    plt.figure(figsize=(12, 8))
    plt.imshow(image_array, cmap="gray")
    # Disegna gli archi originali
    nx.draw_networkx_edges(G, pos, edgelist=original_edges_to_draw, edge_color="blue", width=1)
    # Disegna gli archi aggiunti
    if added_edges_to_draw:
        nx.draw_networkx_edges(G, pos, edgelist=added_edges_to_draw, edge_color="red", width=2, style='dashed')
    # Disegna i nodi
    nx.draw_networkx_nodes(G, pos, node_size=50, node_color="red")
    # Disegna le etichette
    nx.draw_networkx_labels(G, pos, font_size=7, font_color="black")

    plt.axis("off")

    # 13) Salva in PNG
    plt.savefig(output_png_path, format="png", bbox_inches="tight", pad_inches=0)
    plt.close()

    print(f"Immagine con il grafo disegnato salvata in {output_png_path}.")

if __name__ == "__main__":
    main()
