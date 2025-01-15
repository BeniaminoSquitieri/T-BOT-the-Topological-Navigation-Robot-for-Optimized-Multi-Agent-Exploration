# visualization.py

import cv2
from PIL import Image
import networkx as nx
import logging
import numpy as np
import json 
import yaml  # Added import for YAML functionality

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
        json.dump(graph_data, json_file, indent=4, default=numpy_to_python)
    logging.info(f"Topological graph saved in JSON format at {filename}")


def save_topological_graph_on_original_map(original_map, topo_map, png_filename, transformer):
    """
    Overlays the topological graph's nodes, edges, and labels onto the original map and saves the resulting image.
    
    Parameters:
        original_map (numpy.ndarray): The original loaded map (grayscale).
        topo_map (networkx.Graph): The topological graph with nodes and edges.
        png_filename (str): Path to save the resulting image in PNG format.
        transformer (CoordinateTransformer): Object for coordinate transformations.
    """
    # Convert the original map to BGR to allow colored drawings
    original_map_color = cv2.cvtColor(original_map, cv2.COLOR_GRAY2BGR)

    # Define colors in BGR format
    node_color = (0, 0, 255)      # Red for nodes
    edge_color = (0, 255, 0)      # Green for edges
    text_color = (255, 0, 0)      # Blue for text labels

    # Font settings for text labels
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.4
    thickness = 1

    # Draw edges first to ensure nodes are drawn on top of lines
    for edge in topo_map.edges():
        node_start, node_end = edge
        y_start, x_start = node_start
        y_end, x_end = node_end
        cv2.line(original_map_color, (x_start, y_start), (x_end, y_end), edge_color, 1)

    # Draw nodes and their labels
    for node in topo_map.nodes(data=True):
        node_coord = node[0]
        node_attr = node[1]
        y_pixel, x_pixel = node_coord
        label = node_attr.get("label", "")
        
        # Draw the node as a filled circle
        cv2.circle(original_map_color, (x_pixel, y_pixel), 6, node_color, -1)

        if label:
            # Add the label text next to the node
            cv2.putText(original_map_color, label, (x_pixel + 5, y_pixel - 5), font, font_scale, text_color, thickness, cv2.LINE_AA)

    # Save the resulting image using PIL to ensure correct color channels
    Image.fromarray(original_map_color).save(png_filename, format="PNG")
    logging.info(f"Topological graph overlaid on the original map saved at '{png_filename}'")
