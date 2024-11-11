# Directed Chinese Postman Problem on a Directed Graph

## Introduction

This repository provides an implementation of the **Directed Chinese Postman Problem (DCPP)** on a weighted, directed graph. The aim is to determine a minimum-cost path that traverses all edges of a graph at least once, thereby optimizing the route to minimize total travel cost. This is particularly relevant for scenarios like delivery route planning, road maintenance, and robotic navigation for full area coverage.

The **navigation.py** script further extends this by enabling real-time navigation of a TurtleBot4 robot, implementing the DCPP to achieve efficient movement through all predefined waypoints in a real-world environment.

## Project Description

The objective is to solve the Chinese Postman Problem on a directed graph, providing an optimal path that minimizes the cost of traversing all edges. This project applies the solution to enable the TurtleBot4 robot to autonomously navigate a real environment, covering all required areas efficiently.

### Workflow Overview

1. **Graph Reading**: The graph is loaded from a JSON file containing nodes and weighted edges representing, for instance, distances between points.

2. **Eulerian Graph Check**: The code verifies if the graph is Eulerian, meaning there is a path that visits every edge exactly once without unnecessary repetition.

3. **Graph Balancing**: If the graph is not Eulerian, additional edges are added at minimal cost to make it Eulerian.

4. **Eulerian Circuit Search**: Once the graph is balanced, an Eulerian circuit is computed, representing the desired optimal path.

5. **Robot Navigation**: Using the computed path, the TurtleBot4 navigates the environment, moving between waypoints autonomously.

## Dependencies

- **Python 3.x**: The programming language used for implementation.
- **NetworkX**: Python library used for creating, manipulating, and analyzing complex networks. This library is crucial for graph operations and solving the minimum-cost flow problem.
- **ROS2 (Robot Operating System 2)**: For communication and control of the TurtleBot4.
- **TurtleBot4 Navigation Stack**: Provides the navigation functionality for the robot.

## Graph Structure in JSON

The graph JSON file should contain two main sections:

- **nodes**: List of nodes with `x` and `y` coordinates. Each node has a unique label.
- **edges**: List of graph edges. Each edge must have a `from` node and a `to` node and can optionally include a weight. If not provided, the weight is automatically calculated as the Euclidean distance between nodes.

## Navigation Implementation with TurtleBot4

The **navigation.py** script takes the calculated DCPP solution and uses it to guide a TurtleBot4 robot through all waypoints defined in the JSON file. Here is a concise overview of how this is achieved:

1. **Loading the Graph**: The script reads the graph from the provided JSON file and constructs a directed graph using NetworkX. Each node is added with its coordinates, and each edge is weighted by the Euclidean distance.

2. **Calculating the DCPP Route**: The script calculates the optimal route using the DCPP algorithm. If the graph is not Eulerian, it adds edges until the graph can support an Eulerian circuit. The resulting Eulerian path ensures all edges are visited with minimal repetition.

3. **Starting Navigation**: The TurtleBot4 is initialized, and the robot's initial pose is set to the nearest starting node in the graph. The script then guides the robot along the DCPP-calculated route using ROS2 communication protocols.

4. **Real-Time Navigation**: The TurtleBot4 follows the computed path, moving from node to node and reorienting itself appropriately at each point. The ROS2 `TurtleBot4Navigator` is used to manage the robot's motion and ensure it follows the specified waypoints precisely.

5. **Reordering the Path**: The DCPP route is reordered such that the navigation starts at the node closest to the robot's initial position, making it efficient to start and complete the circuit.

### Key Benefits

- **Optimal Edge Coverage**: By solving the DCPP, the robot ensures that all paths are covered efficiently, reducing unnecessary travel and optimizing energy usage.
- **Real-World Application**: The integration with the TurtleBot4 navigation stack enables direct application in real-world scenarios, such as warehouse navigation or search-and-rescue operations.
- **Scalable Solution**: The approach is scalable, suitable for use in environments of varying complexity with directed paths.

## Running the Code

### Preparing the JSON File

Create a JSON file containing the graph with nodes and edges. Ensure that it matches the required format (each node must have coordinates, and edges must indicate `from` and `to` nodes).

### Running the Python Script

Use the following command to execute the navigation script:

```bash
ros2 run nav_pkg navigation.py --start_x <start_x> --start_y <start_y>
```

Provide `<start_x>` and `<start_y>` as the starting coordinates of the TurtleBot4.

This command will initialize the TurtleBot4, compute the DCPP route, and start navigating the environment autonomously.

## CPP Practical Applications

- **Delivery and Waste Collection Services**: Optimizes route planning to minimize overall time and travel costs.
- **Street Maintenance**: Plans efficient routes for road inspections and maintenance.
- **Robotics and Automation**: Programs robots to completely cover an area, ideal for cleaning or security surveillance.

### Extra
In the repository there is also the `dcpp_test.py` script which is an utility used to test the direct chinese postman problem before to take it on the real robot. 

In order to run this utility : 
Navigate to the directory where the script is located

```bash
python dcpp_test.py --start_node <start_node_label>
```