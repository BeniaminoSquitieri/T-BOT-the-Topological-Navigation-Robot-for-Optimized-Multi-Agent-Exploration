# README

## Overview

This directory contains Python scripts for working with graphs and solving the Directed Chinese Postman Problem (DCPP). The DCPP is a route optimization problem where the goal is to find an Eulerian circuit (a closed route visiting every edge of the graph at least once) on a directed graph. These scripts load graph data from a JSON file, perform necessary computations to make the graph Eulerian, and print the resulting route.

---

## Files in the Repository

### **1. `dcpp_test.py`**
This script is the main solver for the Directed Chinese Postman Problem (DCPP). It computes a route on a directed graph that allows a robot or agent to traverse all edges at least once.

#### **Key Features**
- **Graph Loading**: Parses a JSON file containing nodes and edges into a NetworkX directed graph (`DiGraph`).
- **DCPP Route Computation**: Balances the graph by adding necessary edges and computes an Eulerian circuit.
- **Route Printing**: Outputs the sequence of nodes and directions for the computed route.
- **Custom Starting Node and Graph Path**: Accepts a starting node and graph file path as command-line arguments.

#### **Usage**
```bash
python dcpp_test.py --start_node <node_label> --graph_path <path_to_graph.json>
```
- `--start_node`: The label of the node where the route should start.
- `--graph_path`: The path to the JSON file containing the graph data.
For example:

```bash
python3 dcpp_test.py --start_node node_10 --graph_path /home/beniamino/turtlebot4/diem_turtlebot_ws/src/test_python_files/subgraphs/subgraph_2.json

```

#### **Details**
- The JSON file path for the graph is no longer hardcoded. Instead, it is passed as a command-line argument (`--graph_path`) for greater flexibility.
- Ensures the graph is strongly connected before proceeding with computations.
- If the graph is not Eulerian, it adds edges to balance in-degrees and out-degrees.

---

### **2. `dcpp_utils.py`**
This script provides utility functions for working with directed graphs, including graph loading, DCPP route computation, and route printing. It can be used as a library in other projects.

#### **Key Features**
- **Graph Loading (`load_graph`)**:
  - Reads a JSON file containing nodes and edges.
  - Constructs a directed graph (`DiGraph`) with edge weights based on Euclidean distances.

- **DCPP Route Computation (`compute_dcpp_route`)**:
  - Checks if the graph is strongly connected.
  - Balances nodes with unbalanced in-degrees and out-degrees.
  - Solves the DCPP using NetworkX's `min_cost_flow` to find minimal additions to balance the graph.
  - Computes the Eulerian circuit on the modified graph.

- **Route Reordering (`reorder_route`)**:
  - Adjusts the Eulerian circuit to start from a specified node.

- **Route Printing (`print_route`)**:
  - Outputs the sequence of edges and directions for the computed route.

#### **Example Usage**
```python
from dcpp_utils import load_graph, compute_dcpp_route, print_route

# Load the graph from a JSON file
G = load_graph("path_to_graph.json")

# Compute the DCPP route
route = compute_dcpp_route(G)

# Print the route if computation was successful
if route:
    print_route(route, G, "start_node_label")
else:
    print("Unable to compute the DCPP route.")
```

---

### **3. `test_graph_partitioning.py`**
This script is used for partitioning a given graph into smaller subgraphs and saving them along with their visual representations. It is particularly useful in multi-robot applications where the graph represents areas of navigation, and each subgraph needs to be assigned to a different robot.

#### **Key Features**
- **Graph Partitioning**: Loads a complete graph from a JSON file and partitions it into a specified number of subgraphs using a custom partitioning strategy.
- **Visualization**: Saves visual representations of the full graph and each subgraph as PNG images for easy inspection.
- **Directory Management**: Ensures that output directories are cleared and recreated to avoid stale data.

#### **Usage**
```bash
python test_graph_partitioning.py --graph_path <path_to_json> --num_partitions <number> --output_dir <output_directory>
```
- `--graph_path`: The path to the JSON file containing the complete graph.
- `--num_partitions`: The number of subgraphs to partition the full graph into.
- `--output_dir`: The directory where the subgraphs and images will be saved.
For example:


```bash
python3 test_graph_partitioning.py --graph_path /home/beniamino/turtlebot4/diem_turtlebot_ws/src/multi_robot_pkg_navigation/nav_pkg/nav_pkg/navigation_hardware_limitation.json --num_partitions 2 --output_dir ./subgraphs 
```

#### **Details**
- **Graph Loading**: The script reads the graph from a JSON file, where nodes are defined with coordinates (`x`, `y`) and edges define connections between nodes.
- **Partitioning and Saving**:
  - Uses the `load_full_graph`, `partition_graph`, and `save_subgraphs` functions from the `graph_partitioning` module to load, partition, and save the graphs.
  - Each subgraph is saved as a JSON file, and an image representing the graph is also saved for visualization purposes.
- **Resetting Directories**: The script ensures that the output directories are cleared before saving new data, avoiding any confusion due to previous results.

#### **Example Workflow**
1. **Prepare a Graph JSON File**:
   - Create or modify a graph JSON file in the expected format.

2. **Run the Partitioning Script**:
   - Execute the `test_graph_partitioning.py` script to partition the graph and save the resulting subgraphs and visualizations:
     ```bash
     python test_graph_partitioning.py --graph_path path/to/graph.json --num_partitions 3 --output_dir ./subgraphs
     ```

3. **Review Outputs**:
   - The full graph and subgraphs are saved as PNG images in the `graph_images` directory for easy visualization.

#### **JSON Graph Format**
The script expects the graph data in the following JSON format:

```json
{
    "nodes": [
        { "label": "node_1", "x": 0.0, "y": 0.0 },
        { "label": "node_2", "x": 1.0, "y": 1.0 }
    ],
    "edges": [
        { "from": "node_1", "to": "node_2" },
        { "from": "node_2", "to": "node_1" }
    ]
}
```

### **Nodes**
- `label`: A unique identifier for the node.
- `x`, `y`: Coordinates of the node.

### **Edges**
- `from`: The label of the starting node of the edge.
- `to`: The label of the ending node of the edge.

---

## Prerequisites

### **Dependencies**
- Python 3.7+
- Required Python Libraries:
  - `networkx`
  - `argparse`
  - `os`
  - `math`
  - `json`
  - `matplotlib`
  - `shutil`

### **Installation**
Install the required libraries via pip:
```bash
pip install networkx matplotlib
```

---

## Example Workflow

1. **Prepare a Graph JSON File**:
   - Create or modify a graph JSON file to match the expected format.

2. **Run the Solver**:
   - Execute the `dcpp_test.py` script, specifying the starting node and graph path as arguments:
     ```bash
     python dcpp_test.py --start_node node_1 --graph_path path/to/graph.json
     ```

3. **Partition the Graph**:
   - Execute the `test_graph_partitioning.py` script to partition the graph into smaller subgraphs.
     ```bash
     python test_graph_partitioning.py --graph_path path/to/graph.json --num_partitions 3 --output_dir ./subgraphs
     ```

4. **Review the Output**:
   - The script will output the Eulerian circuit starting from the specified node.
   - Partitioned subgraphs and visualizations will be saved for review.

---

## Notes

- Ensure that the graph is **strongly connected** for DCPP computations. If it is not, the script will raise an error during route computation.
- For multi-robot systems, each subgraph should ideally be assigned to a specific robot for navigation purposes.
- The `test_graph_partitioning.py` script can be useful for visualizing and debugging the partitioning of complex graphs.

---


