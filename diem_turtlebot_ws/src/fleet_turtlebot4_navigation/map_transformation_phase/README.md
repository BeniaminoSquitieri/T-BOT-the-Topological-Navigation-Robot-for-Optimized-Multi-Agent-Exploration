# PHASE 1

# Generating a Topological Map from an Occupancy Map

## Introduction
This module aims to transform an occupancy grid map into a topological map. This representation is particularly well-suited for robotics applications, such as autonomous navigation and path planning in complex indoor environments, allowing a robot to understand the spatial relationships between key points in the environment.

## General Description
The process of generating the topological map involves the following steps:

1. **Loading the Occupancy Map**: Load the map image.
2. **Creating the Binary Map**: Convert the image into a binary representation (obstacles/free spaces).
3. **Calculating the Euclidean Distance Map**: Determine the minimum distance of each point from obstacles.
4. **Generating Voronoi Lines**: Identify paths equidistant from obstacles.
5. **Skeletonizing Voronoi Lines**: Reduce the lines to a one-pixel-wide representation.
6. **Creating the Topological Graph**: Construct the graph from the skeleton, with nodes and edges.
7. **Saving Results**: Export images and configuration files for use with ROS2.

## Detailed Steps

### 1. Loading the Occupancy Map
- **Objective**: Load the grayscale map image.
- **Method**: Use OpenCV to read the image and obtain a matrix representing occupancy levels.

### 2. Creating the Binary Map
- **Objective**: Convert the image into a binary map where:
  - 0 (black): represents obstacles.
  - 1 (white): represents free space.
- **Motivation**: A binary representation makes it easier to calculate distances and generate Voronoi lines.

### 3. Calculating the Euclidean Distance Map
- **Objective**: Calculate the minimum distance from each free pixel to the nearest obstacle.
- **Motivation**: The distance map is fundamental for generating Voronoi lines, which represent locations equidistant from obstacles.

### 4. Generating Voronoi Lines
- **Objective**: Identify paths that are equidistant from obstacles, known as Voronoi lines.
- **Method**:
  - Apply a filter to calculate the difference between distance values of neighboring pixels.
  - Points with significant differences indicate locations equidistant from multiple obstacles.
- **Motivation**: Voronoi lines represent central paths, ideal for robotic navigation.

### 5. Skeletonizing Voronoi Lines
- **Objective**: Reduce Voronoi lines to thin, one-pixel-wide lines.
- **Motivation**: Skeletonization helps in identifying nodes and improves the accuracy of topological graph construction.

### 6. Creating the Topological Graph
- **Objective**: Construct a graph representing the map, composed of nodes and edges.
- **Steps**:
  1. **Node Identification**: Identify nodes as intersections and endpoints on the skeleton.
  2. **Node Distribution**: Use the DBSCAN clustering algorithm to distribute nodes evenly.
     - **DBSCAN (Density-Based Spatial Clustering of Applications with Noise)** clusters nearby points based on density. Points are grouped if they are within a certain distance (`eps`) and have a minimum number of neighboring points (`min_samples`). This method is especially useful in areas with high density of nodes that can be merged into a single cluster.

### 7. Saving Results
- **Saved Images**:
  - Binary Map
  - Normalized Distance Map
  - Voronoi Map
  - Voronoi Skeleton
  - Map with Nodes Overlaid

## Requirements

### Required Python Libraries
The following Python libraries are required to run this module:

- `numpy`
- `opencv-python`
- `networkx`
- `scipy`
- `scikit-image`
- `Pillow`
- `PyYAML`

### Execution Environment
- Python 3.x
- ROS2 Humble (for using the map with ROS2)

## Usage Guide

### Installing Dependencies
Run the following command to install all required libraries:
```bash
pip install numpy opencv-python networkx scipy scikit-image Pillow PyYAML
```
### Running the Script
To execute the script, make sure to specify the path of the map image.
```bash
python topological_map.py diem_map.pgm
```

# Viewing Results in RVIZ

After generating the map, you can visualize it in RVIZ to verify its accuracy. Follow the steps below.

## Steps to View the Map

1. **Ensure you have copied or specified the correct path to the generated YAML map.**

2. **Run the following commands in three different terminals to launch the modules required for map visualization and navigation.**

3. **Don't forget to navigate to the map directory**:
- `/turtlebot4/diem_turtlebot_ws/src/map/map_transformation_phase/diem_map_topological`
### Terminal 1 - Start Localization Using the Map
```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=<map_yaml_file_path>
```
In my case:

```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=diem_map.yaml
```

### Terminal 2 - Start the Navigation Module
```bash
ros2 launch turtlebot4_navigation nav2.launch.py
```
### Terminal 3 - Start RVIZ to Visualize the Robot
```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```
> **Note:** Replace `<map_yaml_file_path>` with the path to the generated YAML map.

## Preparing the Environment

- Ensure that the occupancy map image is available and accessible.
- Modify the value of `diem_map.pgm` in the command with your map's path.

## Parameter Configuration
- **image_path**: Specify the path to the map image.

## Output and Visualization of Results

Upon execution, a folder will be created with the name of the map, containing:

### 1. Saved Images
- Skeletonized Map
- Map with Nodes Overlaid
- Binary Map
- Normalized Distance Map
- Voronoi Map

### 2. YAML File
- Contains information such as resolution and map origin for integration with ROS2.

### 3. Topological Graph Representation
- JSON format with the structure of the topological graph.

# PHASE 2

## Graph Visualization File

The `visualize_graph.py` file allows you to visualize the generated topological graph by overlaying it on a background map image and saving the result in both PNG and PGM formats.

### Requirements

- `matplotlib`
- `networkx`
- `Pillow`
- `numpy`

Make sure to install these libraries using the following command:

```bash
pip install matplotlib networkx pillow numpy
```

### Usage Guide

To visualize the topological graph, run the following command, passing the path to the generated JSON file:

```bash
python visualize_graph.py <path_to_json>
```

### Description of `visualize_graph.py`

- **Input**:
  - A JSON file containing the description of the topological graph (nodes and edges) with coordinates based on the RVIZ system.
  - The path to the `diem_map.pgm` image used as the background map.
- **Output**: Two image files of the topological graph overlaid on the map, saved in the `graph` directory:
  - A PNG file for visualization.
  - A PGM file for use with tools that require image formats compatible with ROS.
  
- **Functionality**:
  - Converts graph coordinates from RVIZ to pixel coordinates, using the `origin` and `resolution` values specified in the map's YAML configuration.
  - Draws the topological graph using `matplotlib` and `networkx` on top of the map image.
  - Saves the overlaid graph image in PNG and PGM formats in the `graph` folder.

### Example Usage

Suppose you have generated a JSON file named `navigation_graph.json`. To visualize and overlay the graph on the map as an image:

```bash
python visualize_graph.py navigation_graph.json
```

After execution, two images will be created in the `graph` folder:
- `navigation_graph_graph_map.png`: the graph overlaid on the map in PNG format.
- `navigation_graph_graph_map.pgm`: the graph overlaid on the map in PGM format, useful for loading in ROS2.

