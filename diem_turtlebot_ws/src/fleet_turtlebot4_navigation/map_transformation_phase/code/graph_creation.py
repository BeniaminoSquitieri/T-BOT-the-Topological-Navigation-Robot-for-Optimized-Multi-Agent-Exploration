# graph_creation.py

import networkx as ntx
from sklearn.cluster import DBSCAN
from scipy.spatial import cKDTree
from skimage.draw import line as draw_line
import numpy as np
import logging
from scipy.ndimage import convolve

def check_line_passes_through_skeleton(node1, node2, skeleton, tolerance):
    """
    Checks whether a line between two nodes intersects the skeleton with a specified tolerance.

    This function utilizes Bresenham's line algorithm to determine the pixels that form a straight
    line between two nodes. It then assesses the proportion of these pixels that lie on the skeleton.
    If the proportion of skeleton pixels meets or exceeds the given tolerance, the line is considered
    to pass through the skeleton, indicating a valid navigable path.

    Parameters:
        node1 (tuple): Coordinates of the first node in pixel space (y, x).
        node2 (tuple): Coordinates of the second node in pixel space (y, x).
        skeleton (numpy.ndarray): The skeletonized Voronoi map representing navigable pathways.
        tolerance (float): The minimum proportion of pixels along the line that must belong to the skeleton.
                           Value should be between 0 and 1. Default is 0.06 (6%).

    Returns:
        bool: True if the line passes through the skeleton with the required tolerance, False otherwise.
    """
    y0, x0 = node1
    y1, x1 = node2

    # Generate the pixel coordinates along the line using Bresenham's algorithm
    rr, cc = draw_line(y0, x0, y1, x1)

    # Ensure that the pixel indices are within the bounds of the skeleton map
    valid_indices = (rr >= 0) & (rr < skeleton.shape[0]) & \
                    (cc >= 0) & (cc < skeleton.shape[1])
    
    rr = rr[valid_indices]
    cc = cc[valid_indices]

    if rr.size == 0 or cc.size == 0:
        logging.debug(f"Line from {node1} to {node2} is out of skeleton bounds.")
        return False

    # Determine which pixels along the line are part of the skeleton
    line_skeleton = skeleton[rr, cc]

    # Calculate the proportion of skeleton pixels along the line
    proportion = np.sum(line_skeleton == 1) / len(line_skeleton)

    if proportion >= tolerance:
        logging.debug(f"Line from {node1} to {node2} passes through the skeleton with proportion {proportion:.2f}.")
        return True
    else:
        logging.debug(f"Line from {node1} to {node2} fails proportion {proportion:.2f}.")
        return False


def create_topological_graph_using_skeleton(voronoi_skeleton, config):
    """
    Constructs a topological graph based on the skeletonized Voronoi map.

    This function identifies critical nodes (intersections and endpoints) within the skeleton
    and connects them based on proximity and navigability. It employs clustering to merge nearby
    nodes, ensuring a simplified and efficient graph structure. The resulting graph represents
    the essential navigable pathways within the environment.

    Parameters:
        voronoi_skeleton (numpy.ndarray): The skeletonized Voronoi map indicating navigable paths.
        config (Config): Configuration object containing dynamic parameters such as merge_threshold
                        and max_connection_distance.

    Returns:
        networkx.Graph: The constructed topological graph with nodes and edges representing navigable paths.

    Raises:
        ValueError: If no nodes are detected within the skeletonized map.
    """
    # Initialize an undirected graph to represent the topological map
    topo_map = ntx.Graph()

    # Define a convolution kernel to count the number of neighboring skeleton pixels
    # The center is set to 0 to exclude the current pixel from the count
    kernel = np.array([[1, 1, 1],
                       [1, 0, 1],
                       [1, 1, 1]], dtype=int)

    # Apply convolution to count neighbors for each skeleton pixel
    neighbor_count = convolve(voronoi_skeleton.astype(int), kernel, mode='constant', cval=0)

    # Identify nodes at intersections or endpoints where the number of neighbors is not equal to 2
    node_positions = np.column_stack(np.where((voronoi_skeleton == 1) & (neighbor_count != 2)))

    if node_positions.size == 0:
        logging.error("No nodes detected in the skeleton. Please verify the map cleaning process.")
        raise ValueError("No nodes detected in the skeleton.")

    # Apply DBSCAN clustering to merge nodes that are in close proximity, reducing redundancy
    clustering = DBSCAN(eps=config.merge_threshold, min_samples=1).fit(node_positions)

    # Calculate centroids of each cluster to represent merged nodes
    fused_nodes = []
    nodes_with_labels = []
    for i, cluster_id in enumerate(np.unique(clustering.labels_), start=1):
        cluster_points = node_positions[clustering.labels_ == cluster_id]
        centroid = np.mean(cluster_points, axis=0).astype(int)
        fused_nodes.append(tuple(centroid))
        label = f"node_{i}"
        nodes_with_labels.append((tuple(centroid), {"label": label}))

    logging.info(f"Node fusion completed. Number of nodes after merging: {len(fused_nodes)}")

    # Add fused nodes with labels to the topological graph
    topo_map.add_nodes_from(nodes_with_labels)

    # Construct a KD-Tree for efficient spatial queries of node positions
    node_tree = cKDTree(fused_nodes)

    # Find all unique pairs of nodes that are within the maximum connection distance
    pairs = node_tree.query_pairs(r=config.max_connection_distance)

    logging.info(f"Number of node pairs within {config.max_connection_distance} pixels: {len(pairs)}")

    if not pairs:
        logging.warning("No node pairs found within the maximum connection distance.")

    # Iterate through each pair to determine if a valid connection exists
    for (i, j) in pairs:
        node1 = fused_nodes[i]
        node2 = fused_nodes[j]

        # Check if the line between node1 and node2 passes through the skeleton with the required tolerance
        if check_line_passes_through_skeleton(node1, node2, voronoi_skeleton, config.line_tolerance):
            label1 = topo_map.nodes[fused_nodes[i]]["label"]
            label2 = topo_map.nodes[fused_nodes[j]]["label"]
            distance = np.linalg.norm(np.array(node1) - np.array(node2))
            topo_map.add_edge(label1, label2, distance=distance)
            logging.debug(f"Edge added between {label1} and {label2} with distance {distance:.2f}.")

    return topo_map
