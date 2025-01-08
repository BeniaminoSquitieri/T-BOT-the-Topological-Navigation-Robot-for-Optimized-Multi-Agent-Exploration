# create_topological_map.py

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
create_topological_map.py

Generates a topological map from an occupancy grid map (.pgm) using a YAML configuration file.

Usage:
    python3 create_topological_map.py /path/to/your/config.yaml
    Optionally, you can specify the min_feature_size:
    --min_feature_size <value>

    Example:
    python3 create_topological_map.py config.yaml --min_feature_size 0.59
"""

import os
import argparse
import logging
import shutil
import numpy as np
# Importing custom modules for configuration, coordinate transformation, image processing,
# graph creation, and visualization.
from config import Config
from coordinate_transformer import CoordinateTransformer
import image_processing as img_proc
from graph_creation import create_topological_graph_using_skeleton
from visualization import save_graph_as_json, save_topological_graph_on_original_map

def create_map_directory(map_name):
    """
    Creates a new directory for the map, removing any existing directories with the same name.

    Parameters:
        map_name (str): The name of the map and the corresponding directory.

    Returns:
        str: The absolute path of the created directory.
    """
    # Determine the parent directory relative to the script's location
    parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    
    # Construct the full path for the new map directory
    map_directory = os.path.join(parent_directory, map_name)
    
    # If the directory already exists, remove it to ensure a clean setup
    if os.path.exists(map_directory):
        shutil.rmtree(map_directory)
        logging.info(f"Existing directory '{map_directory}' removed.")
    
    # Create the new directory for the map
    os.makedirs(map_directory)
    logging.info(f"Directory '{map_directory}' created.")
    
    return map_directory

def save_pixel_to_map_transformations(topo_map, filename, transformer):
    """
    Saves the transformation between pixel coordinates and map coordinates to a text file.

    Parameters:
        topo_map (networkx.Graph): The topological graph containing nodes.
        filename (str): The path to the text file where transformations will be saved.
        transformer (CoordinateTransformer): An instance of CoordinateTransformer for coordinate conversions.
    """
    with open(filename, 'w') as file:
        file.write("Transformation between pixel coordinates and map coordinates:\n")
        file.write("Format: (Pixel X, Pixel Y) -> (Map X, Map Y)\n\n")

        # Iterate through each node in the topological graph
        for node in topo_map.nodes(data=True):
            node_coord = node[0]          # Pixel coordinates (y_pixel, x_pixel)
            node_attr = node[1]           # Node attributes, including label
            y_pixel, x_pixel = node_coord
            x_map, y_map = transformer.pixel_to_map(node_coord)
            label = node_attr.get("label", "")
            # Write the transformation details to the file
            file.write(f"{label}: Pixel ({x_pixel}, {y_pixel}) -> Map ({x_map:.2f}, {y_map:.2f})\n")

    logging.info(f"Transformation details saved to '{filename}'")

def process_map(config):
    """
    Coordinates all the necessary steps to process the map and generate the final output files.

    Parameters:
        config (Config): Configuration object containing map parameters.
    """
    # Extract the map name from the image path by removing the file extension
    map_name = os.path.splitext(os.path.basename(config.image_path))[0]

    # Create a dedicated directory for the topological map, appending '_topological' to the map name
    map_directory = create_map_directory(map_name + "_topological")

    # Step 1: Load the original occupancy grid map
    original_map = img_proc.load_map(config.image_path, config.negate)
    # Save the original map as a PNG image for reference
    img_proc.save_as_png(original_map, os.path.join(map_directory, f"{map_name}_original_map.png"))

    # Step 2: Clean the map to remove noise and irrelevant details
    try:
        cleaned_map = img_proc.clean_map(original_map, config, map_directory, map_name)
    except ValueError as e:
        # Log any errors encountered during the cleaning process and terminate processing
        logging.error(f"Error during map cleaning: {e}")
        return

    # Save the cleaned map as a PNG image
    img_proc.save_as_png(cleaned_map, os.path.join(map_directory, f"{map_name}_cleaned_map.png"))

    # Step 3: Create a binary map from the cleaned map
    binary_map = img_proc.create_binary_map(cleaned_map)
    # Save the binary map as a PNG image
    img_proc.save_as_png(binary_map, os.path.join(map_directory, f"{map_name}_cleaned_binary_map.png"))

    # Step 4: Compute the Euclidean distance map from the binary map
    distance_map = img_proc.compute_distance_map(binary_map)
    
    # Normalize the distance map to enhance visualization
    max_distance = np.max(distance_map)
    if max_distance > 0:
        distance_map_normalized = (distance_map / max_distance * 255).astype(np.uint8)
    else:
        # If the maximum distance is zero, create a zeroed image and log a warning
        distance_map_normalized = np.zeros_like(distance_map, dtype=np.uint8)
        logging.warning("Maximum distance in the distance map is 0. Normalized map set to all zeros.")
    
    # Save the normalized distance map as a PNG image
    img_proc.save_as_png(distance_map_normalized, os.path.join(map_directory, f"{map_name}_distance_map.png"))

    # Step 5: Skeletonize the Voronoi lines to extract the fundamental navigational pathways
    voronoi_skeleton = img_proc.skeletonize_voronoi(distance_map_normalized)
    # Save the skeletonized Voronoi map as a PNG image
    img_proc.save_as_png(voronoi_skeleton, os.path.join(map_directory, f"{map_name}_skeleton_voronoi.png"))

    # Initialize the CoordinateTransformer with the cleaned map's dimensions and map parameters
    transformer = CoordinateTransformer(
        image_height=cleaned_map.shape[0],
        resolution=config.resolution,
        origin=config.origin
    )

    # Step 6: Create the topological graph using the skeletonized Voronoi map
    try:
        topo_map = create_topological_graph_using_skeleton(
            voronoi_skeleton,
            config
        )
    except ValueError as e:
        # Log any errors encountered during graph creation and terminate processing
        logging.error(f"Error in creating the topological graph: {e}")
        return

    # Save the topological graph as a JSON file
    save_graph_as_json(topo_map, os.path.join(map_directory, f"{map_name}_topological_graph.json"), transformer)

    # Save the pixel-to-map coordinate transformations to a text file
    save_pixel_to_map_transformations(
        topo_map,
        os.path.join(map_directory, f"{map_name}_pixel_to_map_transformations.txt"),
        transformer
    )

    # Step 8: Overlay the topological graph onto the original map image and save it
    output_map_with_graph_png = os.path.join(map_directory, f"{map_name}_graph_on_original_map.png")
    save_topological_graph_on_original_map(
        original_map,
        topo_map,
        output_map_with_graph_png,
        transformer
    )

    # Log the successful completion of the map processing
    logging.info("Topological map processing completed successfully.")

if __name__ == "__main__":
    # Configure the logging system to display informational messages
    logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

    # Parse command-line arguments to customize the map processing
    parser = argparse.ArgumentParser(description='Generates a topological map from an occupancy grid map.')
    parser.add_argument('config', type=str, help='Path to the YAML configuration file.')
    parser.add_argument('--min_feature_size', type=float, default=0.5, 
                        help='Minimum feature size in meters for morphological operations. Default is 0.5 meters.')
    parser.add_argument('--line_tolerance', type=float, default=0.06, 
                        help='Line tolerance parameter (obsolete).')  # Note: line_tolerance is obsolete
    args = parser.parse_args()

    # Load the configuration using the provided YAML file and min_feature_size parameter
    config = Config(args.config, min_feature_size=args.min_feature_size)
    logging.info("Configuration loaded.")

    # Execute the map processing workflow with the loaded configuration
    process_map(config)
