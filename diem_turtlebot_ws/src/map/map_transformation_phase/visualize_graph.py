import json
import os
import matplotlib.pyplot as plt
import networkx as nx
import sys
from PIL import Image
import numpy as np

# Valori della mappa forniti direttamente (corrispondenti al file diem_map.yaml)
resolution = 0.05  # metri per pixel
origin = (-32.507755, -27.073547)  # coordinate di origine (x, y) della mappa
image_path = "C:\\Users\\Beniamino\\Desktop\\Tesi\\Codice\\diem_turtlebot_ws\\src\\map\\diem_map.pgm"  # percorso completo dell'immagine della mappa

def map_to_pixel(x_map, y_map, origin, resolution, image_height):
    """
    Converte le coordinate mappa RVIZ in coordinate pixel.
    
    Parameters:
        x_map, y_map (float): Coordinate mappa RVIZ.
        origin (tuple): Coordinate di origine della mappa nel frame RVIZ.
        resolution (float): Risoluzione della mappa (metri per pixel).
        image_height (int): Altezza dell'immagine della mappa in pixel.
        
    Returns:
        (int, int): Coordinate pixel.
    """
    x_pixel = int((x_map - origin[0]) / resolution)
    y_pixel = image_height - int((y_map - origin[1]) / resolution)  # Inverti per adattare la mappa
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
    G, pos=node_positions_pixel, with_labels=True, node_size=10, node_color="black",  # Ridotta la dimensione e cambiato il colore dei nodi a nero
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
