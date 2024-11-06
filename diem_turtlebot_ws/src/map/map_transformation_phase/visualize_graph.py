import json
import os
import matplotlib.pyplot as plt
import networkx as nx
import sys

# Check if JSON file path is passed as an argument
if len(sys.argv) < 2:
    print("Usage: python visualize_graph.py <path_to_json>")
    sys.exit(1)

# Load data from JSON file
json_file_path = sys.argv[1]
with open(json_file_path, 'r') as file:
    graph_data = json.load(file)

# Extract nodes and edges
node_positions = {node["label"]: (node["x"], node["y"]) for node in graph_data["nodes"]}
edges = [(edge["from"], edge["to"]) for edge in graph_data["edges"]]

# Initialize the graph and add nodes and edges
G = nx.Graph()
G.add_nodes_from(node_positions.keys())
G.add_edges_from(edges)

# Create the "graph" directory if it doesn't exist
output_directory = "graph"
os.makedirs(output_directory, exist_ok=True)

# Set the output file name to match the JSON file name
json_file_name = os.path.splitext(os.path.basename(json_file_path))[0]
output_file_path = os.path.join(output_directory, f"{json_file_name}.png")

# Draw the graph
plt.figure(figsize=(12, 8))
nx.draw(
    G, pos=node_positions, with_labels=True, node_size=500, node_color="skyblue",
    font_size=8, font_weight="bold", edge_color="gray", linewidths=0.5
)

# Save the graph as an image with the same name as the JSON file
plt.savefig(output_file_path)
plt.close()  # Close the plot to prevent it from displaying

print(f"Graph saved as {output_file_path}")
