#!/usr/bin/env python3

# Import necessary modules
import argparse  # For parsing command-line arguments
import json  # For handling JSON files
import os  # For operating system functions like checking file existence
import matplotlib.pyplot as plt  # For plotting and visualizing the graph on the map
import networkx as nx  # For managing and visualizing the graph structure
import sys  # For exiting the program on critical errors
from PIL import Image  # For loading and manipulating images
import numpy as np  # For numerical computations and array handling
import yaml  # For parsing YAML configuration files

# Function to load map metadata from a YAML file
def load_map_info(yaml_file):
    """
    Loads map information from a YAML file.
    Extracts the following details:
        - 'image_path': Absolute path to the map image (.pgm file)
        - 'resolution': Map resolution in meters/pixel
        - 'origin': The map's origin as a tuple (origin_x, origin_y)
    
    Args:
        yaml_file (str): Path to the YAML file.

    Returns:
        dict: Dictionary containing map metadata.
    """
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)
    # Construct absolute path for the map image
    yaml_dir = os.path.dirname(os.path.abspath(yaml_file))
    image_rel_path = data["image"]
    image_path = os.path.join(yaml_dir, image_rel_path)

    # Extract relevant information from the YAML file
    map_info = {
        "image_path": image_path,
        "resolution": float(data["resolution"]),
        "origin": (float(data["origin"][0]), float(data["origin"][1]))
    }
    return map_info

# Function to convert real-world map coordinates to pixel coordinates
def map_to_pixel(x_map, y_map, origin, resolution, image_height):
    """
    Converts real-world map coordinates to image pixel coordinates.
    Accounts for resolution and origin and inverts the y-axis due to image representation.

    Args:
        x_map (float): X-coordinate in the map.
        y_map (float): Y-coordinate in the map.
        origin (tuple): Map origin as (origin_x, origin_y).
        resolution (float): Map resolution in meters/pixel.
        image_height (int): Height of the map image in pixels.

    Returns:
        tuple: Pixel coordinates (x_pixel, y_pixel).
    """
    x_pixel = int((x_map - origin[0]) / resolution)
    y_pixel = image_height - int((y_map - origin[1]) / resolution)  # Inverts the y-axis
    return x_pixel, y_pixel

# Main function to process arguments, load data, and visualize the graph
def main():
    # Command-line argument parser setup
    parser = argparse.ArgumentParser(description="Visualize a graph on a map, using YAML and JSON input.")
    parser.add_argument("--map_yaml", required=True, help="Path to the map YAML file (containing metadata like image, resolution, origin, etc.).")
    parser.add_argument("--graph_json", required=True, help="Path to the JSON file describing the graph's nodes and edges.")
    parser.add_argument("--additional_edges_json", required=False, help="Path to an optional JSON file with additional edges to include in the graph.")
    args = parser.parse_args()

    # Load map metadata from YAML
    map_info = load_map_info(args.map_yaml)
    image_path = map_info["image_path"]
    resolution = map_info["resolution"]
    origin = map_info["origin"]

    # Load map image from the specified path
    if not os.path.exists(image_path):
        print(f"Error: Map image not found: {image_path}")
        sys.exit(1)
    image = Image.open(image_path).convert("L")  # Load as grayscale
    image_array = np.array(image)  # Convert to NumPy array
    image_height, image_width = image_array.shape

    # Load graph data from the JSON file
    if not os.path.exists(args.graph_json):
        print(f"Error: Graph JSON file not found: {args.graph_json}")
        sys.exit(1)
    with open(args.graph_json, 'r') as file:
        graph_data = json.load(file)

    # Convert node coordinates from map space to pixel space
    node_positions_pixel = {}
    for node in graph_data["nodes"]:
        x_map = node["x"]
        y_map = node["y"]
        label = node["label"]
        x_pixel, y_pixel = map_to_pixel(x_map, y_map, origin, resolution, image_height)
        node_positions_pixel[label] = (x_pixel, y_pixel)

    # Extract edges from the graph data
    edges = []
    for edge in graph_data["edges"]:
        if "source" in edge and "target" in edge:  # Common edge format
            edges.append((edge["source"], edge["target"]))
        elif "from" in edge and "to" in edge:  # Alternative format
            edges.append((edge["from"], edge["to"]))
        else:
            print(f"Warning: Edge with unknown format: {edge}")

    # Create a NetworkX graph and add nodes and edges
    G = nx.Graph()
    G.add_nodes_from(node_positions_pixel.keys())
    G.add_edges_from(edges)

    # Load additional edges from the optional JSON file, if provided
    added_edges = []
    if args.additional_edges_json:
        if not os.path.exists(args.additional_edges_json):
            print(f"Error: Additional edges JSON file not found: {args.additional_edges_json}")
            sys.exit(1)
        with open(args.additional_edges_json, 'r') as file:
            additional_data = json.load(file)
        for edge in additional_data["edges"]:
            if "source" in edge and "target" in edge:
                source = edge["source"]
                target = edge["target"]
                if source in node_positions_pixel and target in node_positions_pixel:
                    G.add_edge(source, target)
                    added_edges.append((source, target))
                else:
                    print(f"Warning: Nodes not found for additional edge: {source} - {target}")

    # Prepare the output directory and filename
    output_directory = "graph"
    os.makedirs(output_directory, exist_ok=True)
    json_file_name = os.path.splitext(os.path.basename(args.graph_json))[0]
    output_png_path = os.path.join(output_directory, f"{json_file_name}_graph_map.png")

    # Identify edges for visualization
    original_edges = set(edges)
    added_edges_set = set(added_edges)
    original_edges_to_draw = list(original_edges)
    added_edges_to_draw = list(added_edges_set)

    # Visualize the map and graph overlay
    plt.figure(figsize=(12, 8))
    plt.imshow(image_array, cmap="gray")  # Display the map in grayscale
    # Draw original edges in blue
    nx.draw_networkx_edges(G, node_positions_pixel, edgelist=original_edges_to_draw, edge_color="blue", width=1)
    # Draw additional edges in red, dashed
    if added_edges_to_draw:
        nx.draw_networkx_edges(G, node_positions_pixel, edgelist=added_edges_to_draw, edge_color="red", width=2, style='dashed')
    # Draw nodes in red
    nx.draw_networkx_nodes(G, node_positions_pixel, node_size=50, node_color="red")
    # Draw node labels in black
    nx.draw_networkx_labels(G, node_positions_pixel, font_size=7, font_color="black")
    plt.axis("off")

    # Save the visualization as a PNG file
    plt.savefig(output_png_path, format="png", bbox_inches="tight", pad_inches=0)
    plt.close()
    print(f"Graph visualization saved at {output_png_path}.")

# Entry point for the script
if __name__ == "__main__":
    main()
