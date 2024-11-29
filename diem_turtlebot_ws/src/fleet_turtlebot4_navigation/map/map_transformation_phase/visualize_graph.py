import json
import os
import matplotlib.pyplot as plt
import networkx as nx
import sys
from PIL import Image
import numpy as np

# Map values provided directly (corresponding to diem_map.yaml file)
resolution = 0.05  # meters per pixel
origin = (-32.507755, -27.073547)  # origin coordinates (x, y) of the map
image_path = "/home/beniamino/turtlebot4/diem_turtlebot_ws/src/multi_robot_pkg_navigation/map/diem_map.pgm"  # full path to the map image

def map_to_pixel(x_map, y_map, origin, resolution, image_height):
    """
    Converts RVIZ map coordinates to pixel coordinates.
    
    Parameters:
        x_map, y_map (float): RVIZ map coordinates.
        origin (tuple): Origin coordinates of the map in RVIZ frame.
        resolution (float): Map resolution (meters per pixel).
        image_height (int): Map image height in pixels.
        
    Returns:
        (int, int): Pixel coordinates.
    """
    x_pixel = int((x_map - origin[0]) / resolution)
    y_pixel = image_height - int((y_map - origin[1]) / resolution)  # Invert for map alignment
    return x_pixel, y_pixel

# Check if JSON file path is passed as an argument
if len(sys.argv) < 2:
    print("Usage: python visualize_graph.py <path_to_json>")
    sys.exit(1)

# Load data from JSON file
json_file_path = sys.argv[1]
with open(json_file_path, 'r') as file:
    graph_data = json.load(file)

# Load the map image to get height and for overlay
image = Image.open(image_path).convert("L")
image_array = np.array(image)
image_height = image_array.shape[0]

# #in the json file we have a dictionary with two keys and each key is characteruzed by a list whose each element is a pair (key, element)
# print("Nodes in graph_data:")
# for node in graph_data["nodes"]:
#     print(f"Label: {node['label']}, X: {node['x']}, Y: {node['y']}")

# print("\nEdges in graph_data:")
# for edge in graph_data["edges"]:
#     print(f"From: {edge['from']} -> To: {edge['to']}")


# Convert each node's RVIZ coordinates to pixel coordinates
node_positions_pixel = {}
for node in graph_data["nodes"]:
    x_pixel, y_pixel = map_to_pixel(node["x"], node["y"], origin, resolution, image_height)
    node_positions_pixel[node["label"]] = (x_pixel, y_pixel)

# Extract edges
edges = [(edge["from"], edge["to"]) for edge in graph_data["edges"]]

# Initialize the graph and add nodes and edges
G = nx.Graph()
G.add_nodes_from(node_positions_pixel.keys())
G.add_edges_from(edges)

# Create the "graph" directory if it doesn't exist
output_directory = "graph"
os.makedirs(output_directory, exist_ok=True)

# Set the output file name to match the JSON file name
json_file_name = os.path.splitext(os.path.basename(json_file_path))[0]
output_png_path = os.path.join(output_directory, f"{json_file_name}_graph_map.png")
output_pgm_path = os.path.join(output_directory, f"{json_file_name}_graph_map.pgm")

# Plot the map and overlay the graph
plt.figure(figsize=(12, 8))
plt.imshow(image_array, cmap="gray")  # Display the original map as background
nx.draw(
    G, pos=node_positions_pixel, with_labels=True, node_size=10, node_color="black",  # Reduced size and changed node color to black
    font_size=8, font_weight="bold", edge_color="blue", linewidths=0.5
)

# Save the combined image as a PNG file
plt.axis("off")
plt.savefig(output_png_path, format="png", bbox_inches="tight", pad_inches=0)
plt.close()

# Convert PNG to PGM
with Image.open(output_png_path) as img:
    img = img.convert("L")  # Ensure grayscale
    img.save(output_pgm_path)

print(f"Graph with overlay saved as {output_pgm_path}")
