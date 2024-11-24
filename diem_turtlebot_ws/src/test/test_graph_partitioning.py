import os
import argparse
import shutil  # Per gestire la rimozione delle directory
import networkx as nx
import matplotlib.pyplot as plt
import sys
sys.path.append('/home/beniamino/turtlebot4/diem_turtlebot_ws/src/map/map_transformation_phase/graph')

from graph_partitioning import load_full_graph, partition_graph, save_subgraphs

# Directory per salvare le immagini dei grafi
IMAGES_DIR = "./graph_images"  # Puoi personalizzare il percorso qui

def reset_directory(directory_path):
    """
    Distrugge e ricrea una directory.
    
    Args:
        directory_path (str): Il percorso della directory da ripristinare.
    """
    if os.path.exists(directory_path):
        shutil.rmtree(directory_path)  # Elimina la directory esistente
    os.makedirs(directory_path)  # Ricrea la directory

def save_graph_as_image(graph, title, image_path):
    """
    Salva un grafo come immagine PNG.
    
    Args:
        graph (nx.Graph): Il grafo da salvare come immagine.
        title (str): Il titolo da visualizzare sull'immagine.
        image_path (str): Il percorso in cui salvare l'immagine.
    """
    # Genera le posizioni dei nodi in base alle coordinate 'x' e 'y'
    pos = {node: (data['x'], data['y']) for node, data in graph.nodes(data=True)}
    
    # Disegna il grafo
    plt.figure(figsize=(10, 8))
    nx.draw(graph, pos, with_labels=True, node_size=700, node_color='lightblue')
    plt.title(title)

    # Salva l'immagine come PNG
    plt.savefig(image_path)
    plt.close()

def main():
    # Configura gli argomenti della riga di comando
    parser = argparse.ArgumentParser(description="Graph Partitioning Simulator")
    parser.add_argument('--graph_path', type=str, required=True, help="Path to the JSON file containing the graph.")
    parser.add_argument('--num_partitions', type=int, required=True, help="Number of subgraphs to partition the graph into.")
    parser.add_argument('--output_dir', type=str, default="./subgraphs", help="Directory to save the subgraph JSON files.")
    args = parser.parse_args()

    # Ripristina le directory di output
    print("Resetting directories...")
    reset_directory(IMAGES_DIR)
    reset_directory(args.output_dir)

    # Step 1: Carica il grafo completo
    print(f"Loading graph from {args.graph_path}...")
    full_graph = load_full_graph(args.graph_path)
    full_graph_image_path = os.path.join(IMAGES_DIR, "full_graph.png")
    print(f"Saving full graph image to {full_graph_image_path}")
    save_graph_as_image(full_graph, title="Full Graph", image_path=full_graph_image_path)

    # Step 2: Partiziona il grafo in sottografi
    print(f"Partitioning graph into {args.num_partitions} subgraphs...")
    subgraphs = partition_graph(full_graph, args.num_partitions)

    # Step 3: Salva i sottografi in file JSON
    print("Saving subgraphs to output directory...")
    subgraph_paths = save_subgraphs(subgraphs, args.output_dir)

    # Step 4: Salva le immagini dei sottografi
    for idx, subgraph in enumerate(subgraphs):
        subgraph_image_path = os.path.join(IMAGES_DIR, f"subgraph_{idx}.png")
        print(f"Saving Subgraph {idx} image to {subgraph_image_path}")
        save_graph_as_image(subgraph, title=f"Subgraph {idx}", image_path=subgraph_image_path)

if __name__ == "__main__":
    main()
