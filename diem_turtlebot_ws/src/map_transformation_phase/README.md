# Map Processing Pipeline

This project presents a comprehensive and modular pipeline for generating topological maps from occupancy grid maps (OGMs). Designed to enhance autonomous navigation, the implementation emphasizes noise reduction, structural preservation, skeletonization, and the creation of a collision-aware topological graph, ensuring reliability and computational efficiency.

## Overview

The pipeline transforms an input **occupancy grid map (OGM)** into a refined and abstracted **topological map** through the following key stages:
1. **Map Cleaning and Preparation**
2. **Binary Map Generation**
3. **Distance Map Calculation**
4. **Skeletonization**
5. **Topological Graph Construction**
6. **Visualization and Validation**

This multi-step approach produces optimized maps tailored for efficient navigation and robust pathfinding, suitable for large-scale autonomous systems.

---

## Features

### Map Cleaning and Preparation
- **Objective**: Eliminate noise and artifacts while preserving critical structural features such as walls and obstacles.
- **Processes**:
  - **Morphological Closing**: Bridges gaps and connects fragmented structures.
  - **Dilation**: Enhances connectivity and emphasizes navigable areas.
  - **Opening**: Removes minor noise artifacts while retaining significant structures.
  - **Erosion**: Refines features and removes residual noise.
- **Outputs**:
  - Cleaned maps with preserved key elements.
  - Intermediate maps for debugging and parameter optimization.

### Binary Map Generation
- Converts the cleaned map into a binary representation where:
  - White pixels denote free space.
  - Black pixels denote obstacles.
- Enables accurate segmentation for subsequent processing steps.

### Distance Map Calculation
- **Objective**: Compute the Euclidean distance from free-space pixels to the nearest obstacles.
- **Purpose**: Provides the foundation for skeletonization.
- **Output**: Normalized distance maps for consistent visualization and enhanced path extraction.

### Skeletonization
- **Objective**: Reduce the environment to a one-pixel-wide representation of navigable pathways.
- **Output**: Skeletonized Voronoi maps delineating essential navigation structures.

### Topological Graph Construction
- **Objective**: Construct a graph where:
  - Nodes represent navigational waypoints.
  - Edges represent collision-free connections between nodes.
- **Features**:
  - Edge validation through Bresenham’s line algorithm to ensure collision-free paths.
  - Automatic connection of isolated components to guarantee graph completeness.
  - Real-world and pixel-based coordinate representations for seamless integration.

### Visualization and Validation
- Overlays nodes, edges, and labels on the original map for verification.
- Saves the graph structure in JSON format, including:
  - Node coordinates in both pixel and real-world units.
  - Edges with source, target, and distance attributes.

---

## Directory Structure

The pipeline generates the following outputs:
```
<map_name>_topological/
├── <map_name>_original_map.png       # Original OGM
├── <map_name>_cleaned_map.png        # Cleaned map
├── <map_name>_cleaned_binary_map.png # Binary map
├── <map_name>_distance_map.png       # Euclidean distance map
├── <map_name>_skeleton_voronoi.png   # Skeletonized map
├── <map_name>_topological_graph.json # Final topological graph in JSON format
├── <map_name>_pixel_to_map_transformations.txt # Coordinate transformations
└── <map_name>_graph_on_original_map.png # Visualization of the graph overlay
```

---

## Code Modules

### 1. `coordinate_transformer.py`
- Manages conversions between pixel and real-world coordinates.
- Key methods:
  - `pixel_to_map`: Transforms pixel coordinates to real-world coordinates.
  - `map_to_pixel`: Converts real-world coordinates to pixel indices.

### 2. `image_processing.py`
- Handles map loading, cleaning, and preprocessing.
- Implements:
  - Morphological transformations.
  - Binary map creation.
  - Distance map computation.
  - Skeletonization of Voronoi diagrams.

### 3. `graph_creation.py`
- Constructs topological graphs from skeletonized maps.
- Ensures:
  - Collision-free paths between nodes.
  - Complete connectivity for seamless navigation.

### 4. `visualization.py`
- Provides utilities for visualizing and saving graphs.
- Outputs include:
  - PNG overlays of graphs on original maps.
  - JSON files describing the graph structure.

### 5. `create_topological_map.py`
- The primary script coordinating all pipeline modules.
- Executes the end-to-end process of map transformation.

---

## Usage

### Prerequisites
- Python 3.x
- Required libraries:
  - `numpy`
  - `opencv-python`
  - `scipy`
  - `sklearn`
  - `networkx`
  - `Pillow`

Install dependencies with:
```bash
pip install -r requirements.txt
```

### Running the Pipeline
To execute the pipeline:
```bash
python3 create_topological_map.py /path/to/config.yaml
```

Optional arguments:
- `--min_feature_size`: Defines the minimum feature size for morphological operations (default: `0.5` meters).
- `--line_tolerance`: Sets the tolerance for edge validation (default: `0.06`).

---

## Validation

To validate the pipeline, intermediate maps and graphs are saved during execution. Key validation steps include:
- **Visual Inspection**:
  - Intermediate maps at each stage are saved, enabling verification of cleaning and preprocessing.
- **Graph Validation**:
  - **Connectivity**: Ensures all nodes are reachable via NetworkX’s `is_connected` function.
  - **Collision-Free Paths**: Validates edges using Bresenham’s algorithm to confirm they avoid obstacles.
- **Pathfinding Tests**:
  - Shortest paths are computed and evaluated for efficiency and accuracy.

---

---
# build_graph.py

This file outlines the detailed methodology employed for constructing a **collision-aware topological graph**, leveraging input nodes defined in a JSON format and a preprocessed occupancy grid map. The approach ensures a precise representation of navigable pathways, optimizing both accuracy and computational efficiency for autonomous systems.

## Process Overview

The graph construction pipeline integrates several advanced techniques to maintain representational fidelity and navigational robustness. The main stages include:

### 1. **Node Parsing**
- Input nodes are ingested from a JSON file structured as follows:
  ```json
  {
      "nodes": [
          { "label": "node_1", "x": 10.0, "y": 15.0 },
          { "label": "node_2", "x": 20.0, "y": 25.0 }
      ]
  }
  ```
- Each node entry comprises a unique label and coordinates in the world reference frame.
- These coordinates are subsequently converted to pixel space using the map's resolution and origin.

### 2. **Graph Initialization**
- Nodes are instantiated within a NetworkX graph object using their parsed labels.
- A k-nearest neighbors (K-NN) approach is implemented to identify candidate edges:
  - Node proximity is calculated using Euclidean distance.
  - Initial edge proposals are subjected to validation.

### 3. **Collision-Free Edge Validation**
- Edges are rigorously evaluated for collision-free navigation using Bresenham’s line algorithm:
  - Each candidate edge is traced pixel-by-pixel against the occupancy grid map.
  - The proportion of free pixels along the edge path is computed.
  - Edges are retained if their free pixel proportion meets or exceeds the **collision tolerance threshold**.

### 4. **Graph Connectivity Adjustment**
- Disconnected components are identified via NetworkX’s connectivity algorithms.
- Additional edges are introduced to ensure full graph connectivity by:
  - Minimizing inter-component node distances.
  - Prioritizing nodes with fewer existing connections to maintain graph simplicity and uniformity.

### 5. **Output Generation**
- The final graph is saved in JSON format with the following structure:
  ```json
  {
      "nodes": [
          { "label": "node_1", "x": 10.0, "y": 15.0 },
          { "label": "node_2", "x": 20.0, "y": 25.0 }
      ],
      "edges": [
          { "source": "node_1", "target": "node_2", "distance": 14.14 }
      ]
  }
  ```
- A visualization overlaying the graph onto the original map is generated, highlighting nodes, edges, and their spatial alignment with the environment.

## Key Parameters

- **Maximum Edge Distance**: Defines the upper limit for edge creation between nodes.
- **Collision Tolerance**: Specifies the minimum proportion of free pixels required for an edge to be considered valid. This is because otherwise the Bresenham alghorithm will never create a line beetwen each pair of node.
- **Maximum Edges per Node**: Regulates the number of connections per node to prevent graph overcomplexity.

## Usage

### Execution
To run the graph construction pipeline, use:
```bash
python3 main.py --map_yaml <path_to_yaml> \
                --graph_json <path_to_json> \
                --max_distance 10.0
```

Additional arguments include:
- `--collision_tolerance`: Adjusts the threshold for edge validation.
- `--max_edges_per_node`: Limits the maximum number of connections per node.

## Outputs

- **Graph JSON**: A detailed file containing all nodes and their validated edges.

---

### Note
In the graph directory there are three different type of graphs. The first one (result_graph.json) is the original graph created by the previous script but it's too big so it has been refined , and so we have the second graph(result_graph_original.json) representitive of the topological map. By the way due to the limit hardware and connection problem of the robot Turtlebot4 to test the graph in real scenarios the second graph should have been refined again , and so the third and last one graph(result_graph_reduced.json) came to life. 


## Contributors
- [Beniamino Squitieri]

For inquiries, contact: [b.squitieri@studenti.unisa.it]

