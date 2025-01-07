# visualization.py


# visualization.py

import cv2
from PIL import Image
import networkx as nx
import logging
import numpy as np
import json 
def numpy_to_python(obj):
    """
    Converts NumPy types to standard Python types.
    
    Parameters:
        obj: NumPy object (array or value).
    
    Returns:
        Object converted to a standard Python type (float or int).
    """
    if isinstance(obj, np.ndarray):
        return obj.tolist()  # If it's a NumPy array, convert to list
    elif isinstance(obj, np.generic):
        return obj.item()  # If it's a generic NumPy object, convert to a Python value
    return obj

def save_waypoints_as_yaml(waypoints, filename):
    """
    Saves waypoints with labels in a YAML file.

    Parameters:
        waypoints (list): List of waypoints (x, y) to save.
        filename (str): Path to the YAML file to save the waypoints.
    """
    waypoints_data = {"waypoints": [{"label": f"node_{i+1}", "x": numpy_to_python(wp[0]), "y": numpy_to_python(wp[1])} 
                                    for i, wp in enumerate(waypoints)]}

    with open(filename, 'w') as yaml_file:
        yaml.dump(waypoints_data, yaml_file, default_flow_style=False)
    logging.info(f"Waypoints saved at {filename}")

def save_graph_as_json(topo_map, filename, transformer):
    """
    Saves the topological graph with nodes and edges in JSON format.

    Parameters:
        topo_map (networkx.Graph): The topological graph with nodes and edges.
        filename (str): Path to the JSON file to save the graph.
        transformer (CoordinateTransformer): Object to transform pixel coordinates to map coordinates.
    """
    nodes = []
    label_to_index = {}
    for i, node in enumerate(topo_map.nodes()):
        x_map, y_map = transformer.pixel_to_map(node)
        node_label = f"node_{i+1}"
        nodes.append({"label": node_label, "x": x_map, "y": y_map})
        label_to_index[node] = node_label

    edges = []
    for i, j in topo_map.edges():
        edges.append({"source": label_to_index[i], "target": label_to_index[j]})

    graph_data = {
        "nodes": nodes,
        "edges": edges
    }

    with open(filename, 'w') as json_file:
        json.dump(graph_data, json_file, indent=4)
    logging.info(f"Topological graph saved in JSON format at {filename}")



def save_topological_graph_on_original_map(original_map, topo_map, png_filename, transformer):
    """
    Sovrappone nodi, archi ed etichette del grafo topologico sulla mappa originale e salva l'immagine risultante.
    
    Parametri:
        original_map (numpy.ndarray): La mappa originale caricata (grayscale).
        topo_map (networkx.Graph): Il grafo topologico con nodi e archi.
        png_filename (str): Percorso per salvare l'immagine risultante in formato PNG.
        transformer (CoordinateTransformer): Oggetto per la trasformazione delle coordinate.
    """
    # Converti la mappa originale in BGR per poter disegnare a colori
    original_map_color = cv2.cvtColor(original_map, cv2.COLOR_GRAY2BGR)

    # Definisci i colori
    node_color = (0, 0, 255)      # Rosso per i nodi
    edge_color = (0, 255, 0)      # Verde per gli archi
    text_color = (255, 0, 0)      # Blu per il testo

    # Impostazioni del font
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.4
    thickness = 1

    # Disegna gli archi prima dei nodi per assicurare che i nodi siano sopra le linee
    for edge in topo_map.edges():
        node_start, node_end = edge
        y_start, x_start = node_start
        y_end, x_end = node_end
        cv2.line(original_map_color, (x_start, y_start), (x_end, y_end), edge_color, 1)

    # Disegna i nodi e le etichette
    for node in topo_map.nodes(data=True):
        node_coord = node[0]
        node_attr = node[1]
        y_pixel, x_pixel = node_coord
        label = node_attr.get("label", "")
        
        # Disegna il nodo come un cerchio
        cv2.circle(original_map_color, (x_pixel, y_pixel), 6, node_color, -1)

        if label:
            # Aggiungi il testo con l'etichetta accanto al nodo
            cv2.putText(original_map_color, label, (x_pixel + 5, y_pixel - 5), font, font_scale, text_color, thickness, cv2.LINE_AA)

    # Salva l'immagine risultante
    Image.fromarray(original_map_color).save(png_filename, format="PNG")
    logging.info(f"Grafo topologico sovrapposto alla mappa originale salvato in '{png_filename}'")

