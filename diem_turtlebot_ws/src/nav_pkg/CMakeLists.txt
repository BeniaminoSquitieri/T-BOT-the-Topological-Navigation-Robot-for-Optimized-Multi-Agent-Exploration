cmake_minimum_required(VERSION 3.8)
project(nav_pkg)

# Trova i pacchetti necessari
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Installazione dei file di lancio
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Installazione dei file JSON dalla cartella "graph"
install(
  DIRECTORY graph/
  DESTINATION share/${PROJECT_NAME}/graph
)

# Installazione degli script Python
install(
  PROGRAMS
    navigation_phase/robot_navigation_node.py
    navigation_phase/navigation.py
    navigation_phase/publish_waypoints.py
    nav_step.py
    obstacle_avoidance.py
    odom.py
    simple_nav.py
  DESTINATION lib/${PROJECT_NAME}
)

# Configura il pacchetto Python
ament_python_install_package(${PROJECT_NAME})

# Configura come un pacchetto ROS 2
ament_package()
