
# Codebase rewrite for ROS2
1. package.xml and CMakeLists.txt have been rewritten for ROS2 compatibility
2. src/collision_publisher.cpp has been rewritten as a class.
3. Launch files rewritten as python files as the dependent launch files are written in python. 
    - gazebo_launch.launch: now additionally launches a ROS-gazebo bridge for communication between ROS and gazebo (standard in ROS2). The bridge configuration file, configs/bridge_config.yaml, has been added.
    - move_base_eband.launch: removed.