import json
import sys
import networkx as nx
from itertools import combinations
import time
from collections import deque

# Robot speed in meters per second
ROBOT_SPEED = 0.31  # m/s

def read_graph(file_path):
    """
    Reads a graph from a JSON file and transforms it into a NetworkX graph object.

    The JSON file must have the following structure:
    {
        "nodes": [
            {"label": "A", "x": 0.0, "y": 0.0},
            {"label": "B", "x": 1.0, "y": 0.0},
            ...
        ],
        "edges": [
            {"source": "A", "target": "B", "distance": 1.0},
            {"source": "B", "target": "C", "distance": 1.0},
            ...
        ]
    }

    Parameters:
        file_path (str): The path to the JSON file containing the graph data.

    Returns:
        networkx.Graph: The constructed graph with nodes and edges.
    """
    try:
        with open(file_path, 'r') as f:
            data = json.load(f)  # Load JSON data from the file
    except FileNotFoundError:
        print(f"Error: The file '{file_path}' was not found.")
        sys.exit(1)  # Exit the program if the file is not found
    except json.JSONDecodeError as e:
        print(f"JSON Decode Error in file '{file_path}': {e}")
        sys.exit(1)  # Exit the program if there's a JSON parsing error
    
    G = nx.Graph()  # Initialize an undirected NetworkX graph

    # Add nodes to the graph
    for node in data['nodes']:
        # Each node is added with its label and positional attributes (x, y)
        G.add_node(node['label'], pos=(node['x'], node['y']))
    
    # Add edges to the graph
    for edge in data['edges']:
        # Each edge connects a source node to a target node with a specified distance
        G.add_edge(edge['source'], edge['target'], distance=edge['distance'])
    
    return G  # Return the constructed graph

def compute_total_distance(edges, G):
    """
    Calculates the total distance traversed given a list of edges.

    This function handles both standard Graphs and MultiGraphs in NetworkX.

    Parameters:
        edges (list): A list of tuples representing edges. Each tuple can be (u, v) for standard graphs
                      or (u, v, key) for MultiGraphs where 'key' identifies the specific edge.
        G (networkx.Graph or networkx.MultiGraph): The graph containing the edges.

    Returns:
        float: The total distance traversed by summing the 'distance' attribute of each edge.
    """
    total_dist = 0.0  # Initialize total distance to zero
    
    for edge in edges:
        if isinstance(G, nx.MultiGraph):
            # Handle MultiGraph where multiple edges can exist between two nodes
            if len(edge) == 3:
                # Edge is a tuple (u, v, key)
                u, v, key = edge
                dist = G[u][v][key].get('distance', 0.0)  # Get distance attribute; default to 0.0 if missing
            else:
                # Edge is a tuple (u, v) without a key
                u, v = edge
                # Attempt to retrieve the first available key for the edge
                keys = list(G[u][v].keys())
                if keys:
                    key = keys[0]
                    dist = G[u][v][key].get('distance', 0.0)
                else:
                    # If no keys are present, assign distance as 0.0
                    dist = 0.0
            total_dist += dist  # Accumulate the distance
        else:
            # Handle standard Graph where only one edge exists between two nodes
            u, v = edge
            dist = G[u][v].get('distance', 0.0)  # Get distance attribute; default to 0.0 if missing
            total_dist += dist  # Accumulate the distance
    
    return total_dist  # Return the total distance traversed

def cpp_traversal_time(G):
    """
    Calculates the total traversal time using the classical approach of the Chinese Postman Problem (CPP)
    for undirected graphs.

    Steps:
    1. Check if the graph is Eulerian (no nodes with odd degree).
       - If yes, compute the Eulerian circuit directly.
    2. If not Eulerian:
       - Construct a complete graph of the odd-degree nodes, weighted by the shortest distances in the original graph.
       - Find a minimum weight matching in this complete graph.
       - Duplicate the edges corresponding to the matching in the original graph (creating an augmented graph G_aug).
       - Compute the Eulerian circuit on the augmented graph.
    3. From the list of edges in the circuit, calculate the total distance and traversal time.

    Parameters:
        G (networkx.Graph): The original undirected graph.

    Returns:
        tuple: (traversal_time in seconds, execution_time in seconds)
    """
    start_time = time.time()  # Record the start time for execution time measurement

    # 1) Identify nodes with odd degrees
    odd_degree_nodes = [node for node, deg in G.degree() if deg % 2 != 0]
    
    if len(odd_degree_nodes) == 0:
        # The graph is already Eulerian; proceed to compute the Eulerian circuit
        try:
            euler_circuit = list(nx.eulerian_circuit(G))  # Generate the Eulerian circuit
        except nx.NetworkXError as e:
            print(f"Error computing Eulerian circuit: {e}")
            sys.exit(1)  # Exit if the circuit cannot be computed

        # Build the list of edges and calculate the total traversed distance
        total_distance = compute_total_distance(euler_circuit, G)
    
    else:
        # 2) The graph is not Eulerian; need to make it Eulerian by duplicating certain edges
        odd_complete = nx.Graph()  # Create a complete graph for odd-degree nodes
        
        # Iterate over all unique pairs of odd-degree nodes to compute shortest paths
        for i, u in enumerate(odd_degree_nodes):
            for v in odd_degree_nodes[i+1:]:
                try:
                    # Compute the shortest path length between nodes u and v
                    dist = nx.shortest_path_length(G, source=u, target=v, weight='distance')
                    odd_complete.add_edge(u, v, weight=dist)  # Add edge to the complete graph with distance as weight
                except nx.NetworkXNoPath:
                    print(f"No path between {u} and {v}; cannot create matching.")
                    continue  # Skip pairs with no available path

        if odd_complete.number_of_edges() == 0:
            # If no edges are present in the complete graph, matching is impossible
            print("Error: Cannot compute a matching among odd-degree nodes.")
            sys.exit(1)
        
        # Find the minimum weight matching in the complete graph
        matching = nx.min_weight_matching(odd_complete, weight='weight')
        
        # Create an augmented graph by duplicating the matched paths
        G_aug = nx.MultiGraph(G)  # Start with a copy of the original graph as a MultiGraph to allow multiple edges
        
        # For each matched pair, duplicate the edges along the shortest path between them
        for (u, v) in matching:
            try:
                path = nx.shortest_path(G, source=u, target=v, weight='distance')  # Find the shortest path
                path_edges = list(zip(path[:-1], path[1:]))  # Convert path to edge tuples
                for (x, y) in path_edges:
                    G_aug.add_edge(x, y, distance=G[x][y]['distance'])  # Duplicate each edge in the path
            except nx.NetworkXNoPath:
                print(f"No path between {u} and {v} during duplication.")
                sys.exit(1)  # Exit if duplication is impossible

        # Verify that the augmented graph is now Eulerian
        if not nx.is_eulerian(G_aug):
            print("Error: The augmented graph is not Eulerian after matching.")
            sys.exit(1)  # Exit if the augmented graph is not Eulerian

        try:
            euler_circuit = list(nx.eulerian_circuit(G_aug))  # Compute the Eulerian circuit on the augmented graph
        except nx.NetworkXError as e:
            print(f"Error computing Eulerian circuit on augmented graph: {e}")
            sys.exit(1)  # Exit if the circuit cannot be computed
        
        # Calculate the total traversed distance in the augmented graph
        total_distance = compute_total_distance(euler_circuit, G_aug)
    
    # Calculate traversal time based on total distance and robot speed
    traversal_time = total_distance / ROBOT_SPEED
    
    # Record the end time and compute execution duration
    end_time = time.time()
    execution_time = end_time - start_time
    
    return traversal_time, execution_time  # Return traversal and execution times

def tsp_traversal_time(G):
    """
    Calculates the total traversal time using an approximation of the Travelling Salesman Problem (TSP).

    Note: This method uses a naive solution and is not efficient for large graphs.

    Parameters:
        G (networkx.Graph): The original undirected graph.

    Returns:
        tuple: (traversal_time in seconds, execution_time in seconds)
    """
    start_time = time.time()  # Record the start time for execution time measurement
    try:
        # Attempt to use the Eulerian circuit as an approximation for the TSP route
        euler_circuit = list(nx.eulerian_circuit(G, source=list(G.nodes)[0]))
        total_distance = compute_total_distance(euler_circuit, G)  # Calculate total traversed distance
    except nx.NetworkXError:
        # If no Eulerian circuit exists, fallback to using CPP to compute traversal
        total_distance, _ = cpp_traversal_time(G)
    traversal_time = total_distance / ROBOT_SPEED  # Calculate traversal time based on distance
    end_time = time.time()  # Record the end time
    execution_time = end_time - start_time  # Calculate execution duration
    return traversal_time, execution_time  # Return traversal and execution times

def dfs_all_edges_both_directions(G, start):
    """
    Performs a Depth-First Search (DFS) that explores all edges in both directions in an undirected graph.

    Parameters:
        G (networkx.Graph): The graph to explore.
        start (str): The starting node for DFS.

    Returns:
        list: A list of tuples representing the traversed edges in the order they were visited,
              including both directions for each edge.
    """
    visited_edges = set()  # Set to keep track of visited edges
    traversal_path = []  # List to store the traversal path

    def dfs(u):
        """
        Recursive helper function to perform DFS.

        Parameters:
            u (str): The current node being visited.
        """
        for v in G.neighbors(u):  # Iterate over all neighbors of the current node
            edge_forward = (u, v)  # Define the forward direction of the edge
            edge_backward = (v, u)  # Define the backward direction of the edge

            if edge_forward not in visited_edges:
                visited_edges.add(edge_forward)  # Mark the edge as visited
                traversal_path.append(edge_forward)  # Add the forward edge to the traversal path
                dfs(v)  # Recursively visit the neighboring node
                traversal_path.append(edge_backward)  # Add the backward edge to return

    dfs(start)  # Initiate DFS from the starting node
    return traversal_path  # Return the complete traversal path

def dfs_traversal_time(G):
    """
    Calculates the total traversal time using a modified Depth-First Search (DFS) that explores all edges
    in both directions.

    Parameters:
        G (networkx.Graph): The graph to explore.

    Returns:
        tuple: (traversal_time in seconds, execution_time in seconds)
    """
    start_time = time.time()  # Record the start time for execution time measurement
    traversal_path = dfs_all_edges_both_directions(G, list(G.nodes)[0])  # Perform DFS traversal
    traversal_time = compute_total_distance(traversal_path, G) / ROBOT_SPEED  # Calculate traversal time
    end_time = time.time()  # Record the end time
    execution_time = end_time - start_time  # Calculate execution duration
    return traversal_time, execution_time  # Return traversal and execution times

def bfs_all_edges_both_directions(G, start):
    """
    Performs a Breadth-First Search (BFS) that explores all edges in both directions in an undirected graph.

    Parameters:
        G (networkx.Graph): The graph to explore.
        start (str): The starting node for BFS.

    Returns:
        list: A list of tuples representing the traversed edges in the order they were visited,
              including both directions for each edge.
    """
    visited_edges = set()  # Set to keep track of visited edges
    traversal_path = []  # List to store the traversal path
    queue = deque([start])  # Initialize a queue with the starting node

    while queue:
        u = queue.popleft()  # Dequeue the next node to visit
        for v in G.neighbors(u):  # Iterate over all neighbors of the current node
            edge_forward = (u, v)  # Define the forward direction of the edge
            edge_backward = (v, u)  # Define the backward direction of the edge

            if edge_forward not in visited_edges:
                visited_edges.add(edge_forward)  # Mark the edge as visited
                traversal_path.append(edge_forward)  # Add the forward edge to the traversal path
                queue.append(v)  # Enqueue the neighboring node for later exploration
                traversal_path.append(edge_backward)  # Add the backward edge to return

    return traversal_path  # Return the complete traversal path

def bfs_traversal_time(G):
    """
    Calculates the total traversal time using a modified Breadth-First Search (BFS) that explores all edges
    in both directions.

    Parameters:
        G (networkx.Graph): The graph to explore.

    Returns:
        tuple: (traversal_time in seconds, execution_time in seconds)
    """
    start_time = time.time()  # Record the start time for execution time measurement
    traversal_path = bfs_all_edges_both_directions(G, list(G.nodes)[0])  # Perform BFS traversal
    traversal_time = compute_total_distance(traversal_path, G) / ROBOT_SPEED  # Calculate traversal time
    end_time = time.time()  # Record the end time
    execution_time = end_time - start_time  # Calculate execution duration
    return traversal_time, execution_time  # Return traversal and execution times

def verify_distances(G):
    """
    Verifies that all edges in the graph have the 'distance' attribute.

    Parameters:
        G (networkx.Graph): The graph to verify.

    Outputs:
        Prints messages indicating whether any edges are missing the 'distance' attribute.
    """
    missing = False  # Flag to indicate if any edge is missing the 'distance' attribute
    for u, v, data in G.edges(data=True):
        if 'distance' not in data:
            print(f"Edge ({u}, {v}) is missing the 'distance' attribute.")
            missing = True
    if not missing:
        print("All edges have the 'distance' attribute.")

def format_time(seconds):
    """
    Formats the time in seconds to a string with two decimal places.

    Parameters:
        seconds (float): The time in seconds.

    Returns:
        str: The formatted time string.
    """
    return f"{seconds:.2f} s"  # Format the time to two decimal places followed by 's'

def main():
    """
    The main function that orchestrates the reading of the graph, verification of attributes,
    computation of traversal times using different algorithms, and displays the results.

    Usage:
        python3 path_validation.py <path_to_json_file>
    """
    if len(sys.argv) != 2:
        print("Usage: python3 path_validation.py <path_to_json_file>")
        sys.exit(1)  # Exit if the correct number of arguments is not provided
    
    file_path = sys.argv[1]  # Get the file path from command-line arguments
    G = read_graph(file_path)  # Read and construct the graph from the JSON file
    
    # Verify that the graph is connected
    if not nx.is_connected(G):
        print("The provided graph is not connected. Please provide a connected graph.")
        sys.exit(1)  # Exit if the graph is not connected
    
    # Verify that all edges have the 'distance' attribute
    verify_distances(G)
    
    # Calculate traversal times for each algorithm
    cpp_time, cpp_exec = cpp_traversal_time(G)  # Chinese Postman Problem traversal time
    tsp_time, tsp_exec = tsp_traversal_time(G)  # Travelling Salesman Problem traversal time
    dfs_time, dfs_exec = dfs_traversal_time(G)  # Depth-First Search traversal time
    bfs_time, bfs_exec = bfs_traversal_time(G)  # Breadth-First Search traversal time
    
    # Print the results in a formatted table
    print(f"{'Algorithm':<25} {'Traversal Time (s)':<30} {'Execution Time (s)':<25}")
    print("-" * 80)
    print(f"{'CPP (Our Algorithm)':<25} {format_time(cpp_time):<30} {cpp_exec:.6f}")
    print(f"{'TSP':<25} {format_time(tsp_time):<30} {tsp_exec:.6f}")
    print(f"{'DFS':<25} {format_time(dfs_time):<30} {dfs_exec:.6f}")
    print(f"{'BFS':<25} {format_time(bfs_time):<30} {bfs_exec:.6f}")

if __name__ == "__main__":
    main()  # Execute the main function when the script is run directly
