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

## **Installation**

### **Dependencies**
To use this package, ensure the following dependencies are installed:
- **ROS2 Humble** (or newer).
- `networkx` for graph-based task computation:
  ```bash
  pip install networkx
  ```
- TurtleBot4 navigation stack:
  ```bash
  sudo apt install ros-humble-turtlebot4-navigation
  ```

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

### **Launching Simulated Navigation**
```bash
ros2 launch fleet_turtlebot4_navigation slave_navigation_simulator_launch.py robot_namespace:=robot1
```

### **Launching Real Navigation**
```bash
ros2 launch fleet_turtlebot4_navigation slave_navigation_launch.py robot_namespace:=robot1
```

---

## **Configuration**

### **Graph File**
The graph file defines waypoints and edges for task allocation. Place your custom graph file in the `config` directory and ensure the launch files reference it correctly.

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

