cmake_minimum_required(VERSION 3.5)
project(multi_nav_pkg)

# Trova pacchetti necessari
find_package(ament_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlebot4_navigation REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Genera i messaggi
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/NavigationStatus.msg"
  DEPENDENCIES std_msgs
)

# Installazione dei pacchetti Python
ament_python_install_package(${PROJECT_NAME})

# Installazione dei file di lancio
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Installazione dei nodi Python
install(
  PROGRAMS
    multi_nav_pkg/master_navigation_node.py
    multi_nav_pkg/backup_master_navigation_node.py
    multi_nav_pkg/slave_navigation_node.py
    multi_nav_pkg/graph_partitioning.py
  DESTINATION lib/${PROJECT_NAME}
)

# Installazione del file package.xml e CMakeLists.txt
install(
  FILES
    package.xml
    CMakeLists.txt
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
