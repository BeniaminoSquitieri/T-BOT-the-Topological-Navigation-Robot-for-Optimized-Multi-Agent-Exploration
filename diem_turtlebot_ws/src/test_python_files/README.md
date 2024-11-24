# README

## Overview

This directory contains Python scripts for working with graphs and solving the Directed Chinese Postman Problem (DCPP). The DCPP is a route optimization problem where the goal is to find an Eulerian circuit (a closed route visiting every edge of the graph at least once) on a directed graph. These scripts load graph data from a JSON file, perform necessary computations to make the graph Eulerian, and print the resulting route.

---

## Files in the Repository

### **1. `dcpp_solver.py`**
This script is the main solver for the Directed Chinese Postman Problem (DCPP). It computes a route on a directed graph that allows a robot or agent to traverse all edges at least once.

#### **Key Features**
- **Graph Loading**: Parses a JSON file containing nodes and edges into a NetworkX directed graph (`DiGraph`).
- **DCPP Route Computation**: Balances the graph by adding necessary edges and computes an Eulerian circuit.
- **Route Printing**: Outputs the sequence of nodes and directions for the computed route.
- **Custom Starting Node**: Accepts a starting node for the route as a command-line argument.

#### **Usage**
```bash
python dcpp_solver.py --start_node <node_label>
```
- `--start_node`: The label of the node where the route should start.

#### **Details**
- The JSON file path for the graph is hardcoded in the script. Modify the `graph_path` variable if needed.
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

## JSON Graph Format

The scripts expect the graph data in the following JSON format:

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

### **Installation**
Install the required libraries via pip:
```bash
pip install networkx
```

---

## Example Workflow

1. **Prepare a Graph JSON File**:
   - Create or modify a graph JSON file to match the expected format.

2. **Run the Solver**:
   - Execute the `dcpp_solver.py` script, specifying the starting node as an argument:
     ```bash
     python dcpp_solver.py --start_node node_1
     ```

3. **Review the Output**:
   - The script will output the Eulerian circuit starting from the specified node.

---

## Notes

- Ensure that the graph is **strongly connected**. If it is not, the script will raise an error during route computation.
- If no starting node is specified, modify the script to handle default starting nodes or random selection.
- To extend the functionality, use `dcpp_utils.py` as a library in custom applications.

---
