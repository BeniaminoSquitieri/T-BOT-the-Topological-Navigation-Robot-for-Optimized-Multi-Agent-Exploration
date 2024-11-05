# MobileRobotsThesis

### STEP 1
# MobileRobotsThesis

## STEP 1

```bash
# Setup environment
source install/setup.bash
ros2 launch turtlebot4_navigation localization.launch.py map:=diem_map_topological/diem_map_topologica_scheletro_nodi.yaml


source install/setup.bash 
ros2 launch turtlebot4_navigation nav2.launch.py

source install/setup.bash 
ros2 launch turtlebot4_viz view_robot.launch.py 
