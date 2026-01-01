cmake_minimum_required(VERSION 3.14.4)
project(jackal_helper)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# DEPENDENCIES
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

# CPP Executables
# add_executable(collision_publisher src/collision_publisher.cpp)

#ament_target_dependencies(
#  collision_publisher
#  rclcpp
#  std_msgs
#  nav_msgs
#  ros_gz_interfaces
#)

#install(TARGETS
#  collision_publisher
#  DESTINATION lib/${PROJECT_NAME}
#)

## Python Executables
install(PROGRAMS
  scripts/gazebo_simulation.py
  scripts/barn_runner.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config launch world 
install(DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
