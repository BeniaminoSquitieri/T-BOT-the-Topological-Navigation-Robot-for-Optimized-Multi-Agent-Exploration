# Advanced Master-Slave Architecture for Robotic Fleet Management

## **Overview**
This package provides a comprehensive implementation of a **Master-Slave architecture** designed for managing robotic fleets in diverse environments. Key highlights include:

- **Dynamic Master Election** powered by the **Bully algorithm**, ensuring robust fault tolerance and automatic recovery during failures.
- Integration with both **simulated testing environments** and **real-world deployment** using **TurtleBot4** robots for seamless transition between development and operation.
- Utilization of **ROS2 communication protocols**, incorporating advanced topic, service, and action handling for efficient inter-robot communication and coordination.

The architecture facilitates dynamic task allocation, real-time state monitoring, and autonomous fault recovery. This makes it ideal for academic research, industrial automation, and other scenarios involving collaborative robotics.

---

## **Features**

### **1. Master-Slave Interaction**
- **Master Responsibilities**:
  - Efficient task distribution to all active slave robots.
  - Real-time monitoring of fleet performance and individual node states.
- **Slave Responsibilities**:
  - Accurate execution of assigned navigation tasks.
  - Continuous communication with the Master via heartbeat signals.

### **2. Bully Election Algorithm**
- Handles seamless Master replacement in case of failure.
- Ensures deterministic conflict resolution using namespace-based prioritization.
- Automatically detects and resolves conflicts, maintaining system stability.

### **3. Flexible Navigation Modes**
- **Simulated Navigation**:
  - Provides a robust testing platform for debugging and validation without hardware dependency.
- **Real TurtleBot4 Navigation**:
  - Uses the advanced `TurtleBot4Navigator` for precise and reliable physical navigation in real-world environments.


---

## **Package Components**

### **Core Nodes**

#### `simulated_slave_navigation_node`
- A dedicated node for virtual testing and debugging.
- Handles simulated waypoint navigation and participates in Master elections using the Bully algorithm.

#### `real_robot_navigation_node`
- Designed for deployment on TurtleBot4 robots.
- Integrates with the TurtleBot4 navigation stack for real-world navigation while adhering to the Master-Slave architecture.

## **File Structure**
```
.
├── CMakeLists.txt                   # Build configuration file for ROS2 package.
├── package.xml                      # ROS2 package manifest describing dependencies and metadata.
├── launch/                          # Directory for all launch files.
│   ├── master_navigation_launch.py  # Launch file for starting the master node.
│   ├── slave_navigation_launch.py   # Launch file for deploying real robot navigation nodes.
│   ├── slave_navigation_simulator_launch.py  # Launch file for simulated navigation nodes.
├── src/                             # Source directory containing the core implementation.
│   ├── bully.py                     # Implementation of the Bully election algorithm.
│   ├── graph_utils.py               # Utility module for managing and parsing graph structures.
│   ├── simulated_slave_navigation_node.py  # Node for simulating navigation tasks.
│   ├── real_robot_navigation_node.py  # Node for real-world navigation with TurtleBot4.
│   └── master/                      # Submodule for Master-specific logic and tools.
│       ├── master_callbacks.py      # Callback functions for the master node.
│       ├── heartbeat_manager.py     # Handles heartbeat signals and timeouts for slaves.
│       ├── waypoint_manager.py      # Manages waypoints and assigns tasks to slaves.
├── config/                          # Configuration files for custom graphs and other settings.
│   └── example_graph.json           # Example JSON file defining navigation graphs.
├── test/                            # Contains testing utilities for the architecture.
│   └── test_python_files/           # Unit and integration tests for core modules.
├── resource/                        # Static files or assets used by the package.
│   └── diem_map.stl                 # Example map file in STL format for simulation.
├── README.md                        # Comprehensive documentation for the project.
├── LICENSE                          # License file describing usage rights.
├── setup.py                         # Python setup script for ROS2 package dependencies.
├── setup.cfg                        # Additional setup configurations.
├── .gitignore                       # Specifies files and directories ignored by Git.

```

---


### **Building the Package**
1. Clone the repository into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url>
   ```
2. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```
3. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

---

## **Usage**


### **Launching Master node**
```bash
ros2 launch fleet_turtlebot4_navigation master_navigation_launch.py
```
### **Launching Simulated Navigation**
```bash
ros2 launch fleet_turtlebot4_navigation slave_navigation_simulator_launch.py robot_namespace:=robot1
```

### Preliminary operations
Before launching the master node and the real slave node, don't forget to connect the Turtlebot4(robot used in the thesis) with the map server and the nav2 stack.
### Terminal 1 - Start Localization Using the Map

```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=<map_yaml_file_path>
```
In my case(going into the directory where I have the map):

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
After that in the other terminals you can run the master slave architecture. 
### **Launching Real Navigation**
```bash
ros2 launch fleet_turtlebot4_navigation slave_navigation_launch.py robot_namespace:=robot1
```

---

## **Configuration**

### **Graph File**
The graph file defines waypoints and edges for task allocation. Ensure the launch files reference to the correct directory where thy're located or alternately specify it through the apposite ros2 launch argument.

#### Example `example_graph.json`:
```json
{
  "nodes": [
    {"id": "node1", "x": 1.0, "y": 2.0},
    {"id": "node2", "x": 2.0, "y": 3.0}
  ],
  "links": [
    {"source": "node1", "target": "node2"}
  ]
}
```

### **Launch Arguments**
- `robot_namespace`: Unique identifier for the robot (e.g., `robot1`).
- `graph_path`: File path to the graph JSON (optional).

---

## **How It Works**

### **Initialization**
- Each Slave node registers itself with the Master during startup.
- The Master maintains a real-time overview of all active nodes, tasks, and graph structures.

### **Task Allocation and Execution**
- Tasks are dynamically assigned by the Master based on the loaded graph structure.
- Simulated and real robots execute tasks, publishing status updates at each step.

### **Failure Recovery**
- Master failures are detected via heartbeat monitoring.
- The Bully algorithm facilitates smooth transition of leadership to a new Master.

---

## **Contributors**
- **Beniamino Squitieri** - Lead Developer

---

## **License**
This project is licensed under the Apache License License. Refer to the `LICENSE` file for more details.

