#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
build_graph.py

This script constructs a navigation graph from a given map and a set of nodes (waypoints).
Unlike the previous version that utilized a binary occupancy grid, this version operates on
a grayscale (normal) map. It interprets varying pixel intensities to determine the level of
occupancy or obstacles, allowing for more nuanced collision detection.

Dependencies:
- OpenCV (`cv2`)
- NumPy (`numpy`)
- PyYAML (`yaml`)
- scikit-image (`skimage`)
- scikit-learn (`sklearn`) [optional]
- NetworkX (`networkx`)

Usage:
    python3 build_graph.py \
        --map_yaml path/to/map.yaml \
        --graph_json path/to/nodes.json \
        --k 20 \
        --max_edges_per_node 7 \
        --occupied_thresh 0.65 \
        --free_thresh 0.196 \
        --collision_tolerance 0.9 \
        --max_distance 15.0 \
        --output_json path/to/output_graph.json \
        --output_image path/to/graph_on_map.png
"""

import argparse          # For handling command-line arguments
import json              # For reading and writing JSON files
import cv2               # OpenCV, for image processing
import numpy as np       # NumPy, for efficient numerical operations on arrays
import yaml              # For reading YAML configuration files
import math              # For basic mathematical operations
import os                # For interacting with the operating system (e.g., file paths)

from skimage.draw import line  # For drawing lines using Bresenham's algorithm

# Attempt to import KDTree from scikit-learn for efficient neighbor searching
try:
    from sklearn.neighbors import KDTree
    USE_SKLEARN = True  # Flag indicating if sklearn is available
except ImportError:
    # If sklearn is not installed, set the flag to False and use an alternative method
    USE_SKLEARN = False

import networkx as nx    # NetworkX for graph management and manipulation

###############################################################################
# Support Functions
###############################################################################

def load_map_info(yaml_file):
    """
    Loads map information from a YAML file.

    Parameters:
        yaml_file (str): Path to the YAML file containing map information.

    Returns:
        dict: Dictionary containing map details, including image path, resolution,
              origin coordinates, negate flag, and thresholds for occupancy.
    """
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)

    map_info = {}

    # Determine the directory of the YAML file to resolve relative image paths
    yaml_dir = os.path.dirname(os.path.abspath(yaml_file))
    image_rel_path = data["image"]
    image_path = os.path.join(yaml_dir, image_rel_path)

    map_info["image_path"] = image_path
    map_info["resolution"] = float(data["resolution"])
    map_info["origin"] = data["origin"]
    map_info["negate"] = int(data.get("negate", 0))
    # Thresholds to determine free and occupied pixels based on grayscale values
    map_info["occupied_thresh"] = float(data.get("occupied_thresh", 0.65))
    map_info["free_thresh"] = float(data.get("free_thresh", 0.196))

    return map_info

def load_occupancy_image(map_info):
    """
    Loads the occupancy map image in grayscale and prepares it for collision detection.

    Parameters:
        map_info (dict): Dictionary containing map information.

    Returns:
        tuple: A tuple containing the grayscale image (numpy.ndarray),
               its height (int), and width (int).
    """
    # Load the image using OpenCV in grayscale mode
    img = cv2.imread(map_info["image_path"], cv2.IMREAD_GRAYSCALE)

    if img is None:
        raise FileNotFoundError(f"Unable to load image: {map_info['image_path']}")

    # If the 'negate' flag is set to 1, invert the pixel values
    if map_info["negate"] == 1:
        img = 255 - img

    height, width = img.shape

    return img, height, width

def load_nodes(json_file):
    """
    Loads nodes (waypoints) from a JSON file.

    Expected JSON structure:
    {
      "nodes": [
        { "label": "node_1", "x": ..., "y": ... },
        ...
      ]
    }

    Parameters:
        json_file (str): Path to the JSON file containing nodes.

    Returns:
        list: List of tuples, each containing (label, x, y) for each node.
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
    Converts world coordinates (x_world, y_world) to pixel coordinates (row, col) in the image.

    Assumes that:
      - origin_x and origin_y are at the bottom-left of the image
      - resolution is in meters/pixel
      - OpenCV image has the origin (0,0) at the top-left

    Conversion formula:
      col = int( (x_world - origin_x) / resolution )
      row = int( map_height - 1 - (y_world - origin_y) / resolution )

    Parameters:
        x_world (float): x-coordinate in the world reference system.
        y_world (float): y-coordinate in the world reference system.
        map_info (dict): Dictionary containing map information.
        map_height (int): Height of the image in pixels.

    Returns:
        tuple: Pixel coordinates (row, col).
    """
    origin_x, origin_y, _ = map_info["origin"]
    resolution = map_info["resolution"]

    # Calculate the floating-point pixel coordinates
    col_f = (x_world - origin_x) / resolution
    row_f = (y_world - origin_y) / resolution

    # Convert to integer pixel coordinates using floor
    col = int(math.floor(col_f))
    row = int(math.floor(map_height - 1 - row_f))  # Invert y-axis for OpenCV

    return row, col

def is_free_pixel(img, row, col, free_thresh):
    """
    Checks if a specified pixel is considered 'free' based on a threshold.

    Parameters:
        img (numpy.ndarray): Grayscale occupancy map.
        row (int): Row index of the pixel.
        col (int): Column index of the pixel.
        free_thresh (float): Threshold to consider a pixel as free.

    Returns:
        bool: True if the pixel is free, False otherwise.
    """
    h, w = img.shape

    # Check if the coordinates are out of image bounds
    if row < 0 or row >= h or col < 0 or col >= w:
        return False  # Out of bounds => considered occupied

    # A pixel is free if its intensity is greater than or equal to the free threshold
    return img[row, col] >= free_thresh*100

def check_collision(img, r1, c1, r2, c2, collision_tolerance=1.0, free_thresh=200):
    """
    Checks if the segment between two pixels crosses occupied areas of the map.

    Uses Bresenham's algorithm to trace the line between (r1, c1) and (r2, c2).
    If the proportion of free pixels along the line is >= collision_tolerance,
    then there is no collision. Otherwise, there is a collision.

    Parameters:
        img (numpy.ndarray): Grayscale occupancy map.
        r1 (int): Starting row.
        c1 (int): Starting column.
        r2 (int): Ending row.
        c2 (int): Ending column.
        collision_tolerance (float, optional): Minimum percentage of free pixels required
                                              to consider the segment clear (no collision).
                                              Default is 1.0 (100%).
        free_thresh (float, optional): Threshold to consider a pixel as free. Default is 0.196.

    Returns:
        bool: True if there is a collision, False otherwise.
    """
    # Trace the line between the two points using Bresenham's algorithm
    rr, cc = line(r1, c1, r2, c2)  # Lists of row and column indices along the line

    total_pixels = len(rr)
    free_pixels = 0

    # Iterate over each pixel along the line
    for i in range(total_pixels):
        if is_free_pixel(img, rr[i], cc[i], free_thresh):
            free_pixels += 1

    # Calculate the proportion of free pixels
    free_ratio = free_pixels / total_pixels

    # Determine if the segment is clear based on the collision tolerance
    if free_ratio >= collision_tolerance:
        return False  # No collision
    else:
        return True   # Collision exists

def compute_distance(x1, y1, x2, y2):
    """
    Calculates the Euclidean distance between two points in the plane.

    Parameters:
        x1 (float): x-coordinate of the first point.
        y1 (float): y-coordinate of the first point.
        x2 (float): x-coordinate of the second point.
        y2 (float): y-coordinate of the second point.

    Returns:
        float: Euclidean distance between the two points.
    """
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distance

###############################################################################
# Modular Functions
###############################################################################

def build_initial_graph(nodes, map_info, img, h, k, max_edges_per_node, collision_tolerance, max_distance):
    """
    Builds the initial graph based on K-Nearest Neighbors (K-NN),
    taking into account collisions and maximum distance.

    Additionally, when multiple neighbors have the same distance, prefers connecting
    to the node that is farther from the map's origin.

    Parameters:
        nodes (list): List of nodes as tuples (label, x, y).
        map_info (dict): Dictionary containing map information.
        img (numpy.ndarray): Grayscale occupancy map.
        h (int): Image height in pixels.
        k (int): Number of neighbors to consider for each node (K-NN).
        max_edges_per_node (int): Maximum number of edges per node after collision checks.
        collision_tolerance (float): Minimum percentage of free pixels required to consider a segment clear.
        max_distance (float): Maximum Euclidean distance allowed to connect two nodes.

    Returns:
        list: List of initial edges as dictionaries {"source": ..., "target": ..., "distance": ...}.
    """
    # Extract node coordinates into a NumPy array for processing
    coords = np.array([[nd[1], nd[2]] for nd in nodes], dtype=np.float32)  # Shape (N, 2)
    labels = [nd[0] for nd in nodes]  # List of node labels

    if USE_SKLEARN:
        # If sklearn is available, use KDTree for efficient neighbor searching
        tree = KDTree(coords)  # Build the KD-Tree
        # Perform a K-NN query for each node, including itself (k+1)
        all_neighbors_idx = tree.query(coords, k=k+1, return_distance=True)
        all_distances = all_neighbors_idx[0]  # Distance matrix
        all_indices = all_neighbors_idx[1]    # Neighbor index matrix
    else:
        # If sklearn is not available, use a less efficient all-pairs approach
        N = len(coords)  # Total number of nodes
        all_indices = []    # List for neighbor indices
        all_distances = []  # List for neighbor distances

        for i in range(N):
            dlist = []  # Temporary list for distances and neighbor indices

            for j in range(N):
                if i == j:
                    continue  # Skip if it's the same node

                # Calculate Euclidean distance between node i and node j
                dist = compute_distance(coords[i][0], coords[i][1],
                                        coords[j][0], coords[j][1])
                dlist.append((dist, j))  # Add the tuple (distance, index)

            # Sort the list by ascending distance
            dlist.sort(key=lambda x: x[0])

            # Take the first k neighbors
            knn = dlist[:k]

            # Extract only the indices and distances
            neighbor_indices = [x[1] for x in knn]
            neighbor_dists = [x[0] for x in knn]

            # For consistency with KDTree, add itself as the first neighbor
            neighbor_indices = [i] + neighbor_indices
            neighbor_dists = [0.0] + neighbor_dists

            # Add the indices and distances to their respective lists
            all_indices.append(neighbor_indices)
            all_distances.append(neighbor_dists)

        # Convert the lists to NumPy arrays for efficiency
        all_indices = np.array(all_indices, dtype=int)       # Shape (N, k+1)
        all_distances = np.array(all_distances, dtype=float)  # Shape (N, k+1)

    # Initialize an adjacency list for each node
    adjacency_list = [[] for _ in range(len(nodes))]

    # Extract origin coordinates for secondary sorting
    origin_x, origin_y, _ = map_info["origin"]

    # Iterate over each node to determine its edges
    for i in range(len(nodes)):
        neighbor_idx_row = all_indices[i]      # Neighbor indices for node i
        neighbor_dist_row = all_distances[i]   # Neighbor distances for node i

        candidate_list = []  # Temporary list for candidate edges

        # Iterate over each neighbor (skip the first if it's the same node)
        for jj in range(1, len(neighbor_idx_row)):
            j = neighbor_idx_row[jj]          # Neighbor index
            dist_ij = neighbor_dist_row[jj]   # Distance from node i to node j

            if j == i:
                continue  # Skip if it's the same node (already handled)

            # Check if the distance is within the maximum allowed
            if dist_ij > max_distance:
                continue  # Skip this connection if the distance exceeds the limit

            # Extract node information for nodes i and j
            label_i, x_i, y_i = nodes[i]
            label_j, x_j, y_j = nodes[j]

            # Convert world coordinates to pixel coordinates for both nodes
            r1, c1 = world_to_map(x_i, y_i, map_info, h)
            r2, c2 = world_to_map(x_j, y_j, map_info, h)

            # Retrieve threshold values for determining free pixels
            free_thresh = map_info["free_thresh"]

            # Check for collision along the path between the two nodes
            has_collision = check_collision(
                img, r1, c1, r2, c2,
                collision_tolerance=collision_tolerance,
                free_thresh=free_thresh
            )

            # If there is NO collision, add the neighbor as a candidate
            if not has_collision:
                # Calculate the distance of neighbor node from origin for secondary sorting
                node_distance_from_origin = compute_distance(x_j, y_j, origin_x, origin_y)

                # Append a tuple with (neighbor index, distance, node_distance_from_origin)
                candidate_list.append((j, dist_ij, node_distance_from_origin))
            else:
                # Debug message indicating a collision was detected
                print(f"Collision detected between '{label_i}' and '{label_j}' with distance {dist_ij:.2f}")

        # Sort the candidates by ascending distance, then by descending distance from origin
        # This ensures that for neighbors with the same distance, the one farther from origin is preferred
        candidate_list.sort(key=lambda x: (x[1], -x[2]))

        # Keep only the first 'max_edges_per_node' edges for this node
        chosen = candidate_list[:max_edges_per_node]

        # Assign the chosen edges to the adjacency list for node i
        adjacency_list[i] = [(j, dist) for (j, dist, _) in chosen]

    # Initialize a list to store the final edges
    edges_out = []

    # Build the edge list based on the adjacency list
    for i in range(len(nodes)):
        label_i = nodes[i][0]  # Label of node i

        for (j, dist_ij) in adjacency_list[i]:
            label_j = nodes[j][0]  # Label of node j

            # Create a dictionary representing the edge
            edge_dict = {
                "source": label_i,      # Source node
                "target": label_j,      # Target node
                "distance": dist_ij     # Distance between nodes
            }

            # Add the dictionary to the edge list
            edges_out.append(edge_dict)

    return edges_out  # Returns the list of initial edges

def connect_disconnected_components(G, nodes, map_info, img, h, collision_tolerance, max_distance, max_edges_per_node):
    """
    Checks the connectivity of the graph and connects disconnected components
    by selecting pairs of nodes with the minimum Euclidean distance, preferring nodes with fewer edges,
    within a maximum distance limit and edge per node limit.

    Additionally, when multiple pairs have the same distance, prefers connecting to nodes that are farther from the origin.

    Parameters:
        G (networkx.Graph): Existing NetworkX graph.
        nodes (list): List of nodes as tuples (label, x, y).
        map_info (dict): Dictionary containing map information.
        img (numpy.ndarray): Grayscale occupancy map.
        h (int): Image height in pixels.
        collision_tolerance (float): Minimum percentage of free pixels required.
        max_distance (float): Maximum Euclidean distance allowed to connect two nodes.
        max_edges_per_node (int): Maximum number of edges per node.

    Returns:
        list: List of added edges as dictionaries {"source": ..., "target": ..., "distance": ...}.
    """
    # Check if the graph is already fully connected
    if nx.is_connected(G):
        print("The graph is already connected.")
        return []  # No edges added

    print("The graph is disconnected. Attempting to connect the components.")

    # Identify all connected components in the graph
    components = list(nx.connected_components(G))  # List of sets, each set is a connected component
    edges_added = []  # List to store added edges

    # Get the current degree of each node in the graph
    degrees = dict(G.degree())  # Dictionary {node: degree}

    # Extract origin coordinates for secondary sorting
    origin_x, origin_y, _ = map_info["origin"]

    # Continue adding edges until the graph is fully connected
    while len(components) > 1:
        candidate_pairs = []  # Temporary list for candidate pairs to connect components

        # Find all possible pairs of components to connect
        for i in range(len(components)):
            for j in range(i + 1, len(components)):
                component_a = components[i]  # First component
                component_b = components[j]  # Second component

                # Iterate over all nodes in the first component
                for node_a in component_a:
                    # Iterate over all nodes in the second component
                    for node_b in component_b:
                        # Extract node data (label, x, y)
                        node_a_data = next(nd for nd in nodes if nd[0] == node_a)
                        node_b_data = next(nd for nd in nodes if nd[0] == node_b)

                        # Calculate the Euclidean distance between the two nodes
                        distance = compute_distance(
                            node_a_data[1], node_a_data[2],
                            node_b_data[1], node_b_data[2]
                        )

                        # Check if the distance is within the maximum allowed
                        if distance > max_distance:
                            continue  # Skip this pair of nodes

                        # Check if adding the edge would exceed the edge limit per node
                        if degrees.get(node_a, 0) >= max_edges_per_node or degrees.get(node_b, 0) >= max_edges_per_node:
                            continue  # Skip this pair of nodes

                        # Convert world coordinates to map (pixel) coordinates
                        r1, c1 = world_to_map(node_a_data[1], node_a_data[2], map_info, h)
                        r2, c2 = world_to_map(node_b_data[1], node_b_data[2], map_info, h)

                        # Retrieve threshold values for determining free pixels
                        free_thresh = map_info["free_thresh"]

                        # Check for collision along the path between the two nodes
                        has_collision = check_collision(
                            img, r1, c1, r2, c2,
                            collision_tolerance=collision_tolerance,
                            free_thresh=free_thresh
                        )

                        # If there is NO collision, add the pair to the candidate list
                        if not has_collision:
                            # Get the distance of node_b from origin for secondary sorting
                            node_b_distance_from_origin = compute_distance(node_b_data[1], node_b_data[2], origin_x, origin_y)

                            # Add a tuple with (distance, node_b_distance_from_origin, node_a, node_b)
                            candidate_pairs.append((distance, node_b_distance_from_origin, node_a, node_b))

        # If there are no candidate pairs, it's impossible to connect the remaining components
        if not candidate_pairs:
            print("Unable to connect all components without collisions or within the maximum distance.")
            break  # Exit the loop to avoid infinite looping

        # Sort the candidate pairs:
        # 1. By ascending distance
        # 2. For equal distances, by descending node_b_distance_from_origin
        candidate_pairs.sort(key=lambda x: (x[0], -x[1]))

        # Select the best pair (minimum distance, prefer farther from origin)
        best_pair = candidate_pairs[0]
        distance, node_b_distance_from_origin, node_a, node_b = best_pair

        # Add the edge to the graph
        G.add_edge(node_a, node_b, distance=distance)

        # Create a dictionary representing the added edge
        edge_dict = {
            "source": node_a,    # Source node
            "target": node_b,    # Target node
            "distance": distance  # Distance between nodes
        }
        edges_added.append(edge_dict)  # Add the edge to the list of added edges

        # Update the degrees of the involved nodes
        degrees[node_a] = degrees.get(node_a, 0) + 1
        degrees[node_b] = degrees.get(node_b, 0) + 1

        # Print an informational message
        print(f"Added edge between '{node_a}' and '{node_b}' to connect the components.")

        # Conditional print for analyzing specific connections between node_19 and node_23
        if ((node_a == "node_19" and node_b == "node_23") or
            (node_a == "node_23" and node_b == "node_19")):
            print(f"Specific analysis between '{node_a}' and '{node_b}':")
            print(f"  Euclidean Distance: {distance:.2f}")
            print(f"  Collision: No")  # Since has_collision is False

        # Update the list of connected components after adding the edge
        components = list(nx.connected_components(G))

    # After attempting to connect components, check the final connectivity of the graph
    if nx.is_connected(G):
        print("The graph has been connected.")
    else:
        print("The graph is not fully connected. Some components remain separate.")

    return edges_added  # Returns the list of added edges

def draw_graph(img_color, nodes, edges, map_info, h):
    """
    Draws the edges of the graph on the colored image.

    Parameters:
        img_color (numpy.ndarray): Colored map image in BGR format.
        nodes (list): List of nodes as tuples (label, x, y).
        edges (list): List of edges as dictionaries {"source": ..., "target": ..., "distance": ...}.
        map_info (dict): Dictionary containing map information.
        h (int): Image height in pixels.
    """
    for edge in edges:
        label_i = edge["source"]  # Source node label
        label_j = edge["target"]  # Target node label

        # Find the corresponding node data for the labels
        node_i = next(nd for nd in nodes if nd[0] == label_i)
        node_j = next(nd for nd in nodes if nd[0] == label_j)

        # Convert world coordinates to pixel coordinates for both nodes
        r1, c1 = world_to_map(node_i[1], node_i[2], map_info, h)
        r2, c2 = world_to_map(node_j[1], node_j[2], map_info, h)

        # Draw a red line between the two nodes on the colored image
        cv2.line(img_color, (c1, r1), (c2, r2), (0, 0, 255), 1)

def draw_nodes_and_labels(img_color, nodes, map_info, h):
    """
    Draws the nodes and their labels on the colored image.

    Parameters:
        img_color (numpy.ndarray): Colored map image in BGR format.
        nodes (list): List of nodes as tuples (label, x, y).
        map_info (dict): Dictionary containing map information.
        h (int): Image height in pixels.
    """
    for node in nodes:
        label, x, y = node  # Extract node label and coordinates

        # Convert world coordinates to pixel coordinates
        r, c = world_to_map(x, y, map_info, h)

        # Draw a green circle at the point (c, r) on the colored image
        cv2.circle(img_color, (c, r), radius=5, color=(0, 255, 0), thickness=-1)

        # Add the node label next to the circle
        # Set text properties
        font = cv2.FONT_HERSHEY_SIMPLEX  # Font type
        font_scale = 0.4                  # Font scale
        font_thickness = 1                # Text thickness

        # Calculate the size of the text to position it correctly
        text_size, _ = cv2.getTextSize(label, font, font_scale, font_thickness)

        # Calculate text coordinates with a small offset to avoid overlap
        text_x = c + 6  # Horizontal offset
        text_y = r - 6  # Vertical offset

        # Put the label text on the image
        cv2.putText(img_color, label, (text_x, text_y), font, font_scale, (255, 0, 0), font_thickness, cv2.LINE_AA)

def save_graph_json(nodes, edges, output_json):
    """
    Saves the graph to a JSON file with a specific structure.

    JSON Structure:
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

    Parameters:
        nodes (list): List of nodes as tuples (label, x, y).
        edges (list): List of edges as dictionaries {"source": ..., "target": ..., "distance": ...}.
        output_json (str): Path to the output JSON file.
    """
    # Create a dictionary for the resulting graph
    result_graph = {
        "nodes": [
            {
                "label": nd[0],  # Node label
                "x": nd[1],      # x-coordinate in the world
                "y": nd[2]       # y-coordinate in the world
            } for nd in nodes
        ],
        "edges": [
            {
                "source": edge["source"],    # Source node
                "target": edge["target"],    # Target node
                "distance": edge["distance"] # Distance between nodes
            } for edge in edges
        ]
    }

    # Open the output JSON file in write mode
    with open(output_json, 'w') as f:
        json.dump(result_graph, f, indent=2)  # Write the dictionary to JSON with indentation

    # Print a confirmation message
    print(f"Graph successfully built and saved in {output_json}.")

def save_graph_image(img_color, output_image):
    """
    Saves the image with the drawn graph.

    Parameters:
        img_color (numpy.ndarray): Map image with the graph drawn on it (in BGR format).
        output_image (str): Path to the output image file.
    """
    # Save the image using OpenCV
    cv2.imwrite(output_image, img_color)

    # Print a confirmation message
    print(f"Image with the drawn graph saved in {output_image}.")

def remove_unconnected_nodes(nodes, edges):
    """
    Removes nodes that are not connected to any other nodes.

    Parameters:
        nodes (list): List of nodes as tuples (label, x, y).
        edges (list): List of edges as dictionaries {"source": ..., "target": ..., "distance": ...}.

    Returns:
        tuple: A tuple containing the filtered list of nodes and the unchanged list of edges.
    """
    connected_nodes = set()
    for edge in edges:
        connected_nodes.add(edge["source"])
        connected_nodes.add(edge["target"])

    # Filter nodes to include only those with at least one edge
    filtered_nodes = [node for node in nodes if node[0] in connected_nodes]

    # Determine which nodes have been removed
    removed_nodes = [node for node in nodes if node[0] not in connected_nodes]
    if removed_nodes:
        print("Removed nodes because they are not connected:")
        for node in removed_nodes:
            print(f"  {node[0]}")

    return filtered_nodes, edges  # The edges are already connected to the filtered nodes

###############################################################################
# Main Function
###############################################################################

def main():
    """
    Main function that coordinates all operations to build the navigation graph,
    handle collisions, connect disconnected components, and save the results
    in JSON files and images.
    """
    # Create a parser to handle command-line arguments
    parser = argparse.ArgumentParser(
        description="Builds a navigation graph with collision tolerance and saves the image with the drawn graph."
    )

    # Add necessary arguments to the parser
    parser.add_argument("--map_yaml", type=str, required=True,
                        help="Path to the .yaml map file (Occupancy Grid).")
    parser.add_argument("--graph_json", type=str, required=True,
                        help="Path to the .json file containing nodes (waypoints).")
    parser.add_argument("--k", type=int, default=6,
                        help="Number of neighbors to consider for each node (K-NN). Must be >= max_edges_per_node.")
    parser.add_argument("--max_edges_per_node", type=int, default=4,
                        help="Maximum number of edges to retain per node after collision checks.")
    parser.add_argument("--output_json", type=str, default="output_graph.json",
                        help="Name of the output JSON file with the complete graph.")
    parser.add_argument("--output_image", type=str, default="graph_on_map.png",
                        help="Name of the output image file with the drawn graph.")
    # Removed the free_value_thresh parameter since we're using thresholds from the YAML
    parser.add_argument("--collision_tolerance", type=float, default=1.0,
                        help="Percentage [0..1] of free pixels required to consider a segment clear (no collision).")
    parser.add_argument("--max_distance", type=float, required=True,
                        help="Maximum allowed Euclidean distance between two nodes to connect them via an edge.")

    # Parse the arguments provided by the user
    args = parser.parse_args()

    # Ensure that the number of neighbors 'k' is at least equal to 'max_edges_per_node'
    if args.k < args.max_edges_per_node:
        print(f"Error: the --k parameter ({args.k}) must be >= --max_edges_per_node ({args.max_edges_per_node}).")
        return  # Exit the main function

    # 1) Load the map information from the YAML file
    map_info = load_map_info(args.map_yaml)

    # 2) Load the grayscale occupancy map image
    img, h, w = load_occupancy_image(map_info)

    # Convert the image to BGR format to allow colored drawings
    img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # 3) Load the nodes (waypoints) from the JSON file
    nodes = load_nodes(args.graph_json)
    # nodes is a list of tuples: [(label, x, y), ...]

    # 4) Build the initial graph based on K-NN, collisions, and distance
    initial_edges = build_initial_graph(
        nodes=nodes,
        map_info=map_info,
        img=img,
        h=h,
        k=args.k,
        max_edges_per_node=args.max_edges_per_node,
        collision_tolerance=args.collision_tolerance,
        max_distance=args.max_distance
    )

    # 5) Draw the initial edges on the colored image
    draw_graph(img_color, nodes, initial_edges, map_info, h)

    # 6) Create an undirected NetworkX graph
    G = nx.Graph()
    G.add_nodes_from([node[0] for node in nodes])  # Add all nodes to the graph
    G.add_edges_from([(edge["source"], edge["target"]) for edge in initial_edges])  # Add initial edges

    # 7) Check and connect any disconnected components in the graph
    edges_added = connect_disconnected_components(
        G=G,
        nodes=nodes,
        map_info=map_info,
        img=img,
        h=h,
        collision_tolerance=args.collision_tolerance,
        max_distance=args.max_distance,
        max_edges_per_node=args.max_edges_per_node
    )

    # 8) Draw the added edges on the colored map
    for edge in edges_added:
        source, target, distance = edge["source"], edge["target"], edge["distance"]
        node_source = next(nd for nd in nodes if nd[0] == source)
        node_target = next(nd for nd in nodes if nd[0] == target)

        # Convert world coordinates to pixel coordinates for both nodes
        r1, c1 = world_to_map(node_source[1], node_source[2], map_info, h)
        r2, c2 = world_to_map(node_target[1], node_target[2], map_info, h)

        # Draw a red line between the added nodes
        cv2.line(img_color, (c1, r1), (c2, r2), (0, 0, 255), 1)  # Red color (BGR: 0,0,255)

    # 9) Draw the nodes and their labels on the colored image
    draw_nodes_and_labels(img_color, nodes, map_info, h)

    # 10) Prepare the final edge list by combining initial and added edges
    final_edges = initial_edges + edges_added

    # -- Remove any duplicates: (source, target) and (target, source) should be the same edge --
    unique_edges_set = set()
    unique_edges_list = []

    for e in final_edges:
        s = e["source"]
        t = e["target"]
        d = e["distance"]

        # Normalize the order of labels to ensure uniqueness (min, max)
        if s > t:
            s, t = t, s

        edge_tuple = (s, t, d)

        if edge_tuple not in unique_edges_set:
            unique_edges_set.add(edge_tuple)

            edge_dict = {
                "source": s,       # Source node
                "target": t,       # Target node
                "distance": d      # Distance between nodes
            }
            unique_edges_list.append(edge_dict)

    final_edges = unique_edges_list

    # 11) Remove nodes that are not connected to any other nodes
    filtered_nodes, final_edges = remove_unconnected_nodes(nodes, final_edges)

    # 12) Save the final graph to a JSON file
    save_graph_json(filtered_nodes, final_edges, args.output_json)

    # 13) Save the image with the drawn graph
    save_graph_image(img_color, args.output_image)

###############################################################################
# Entry Point
###############################################################################

if __name__ == "__main__":
    main()
