cmake_minimum_required(VERSION 2.8.3)
project(robot_hardware)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  controller_manager
  roscpp
  control_msgs
  trajectory_msgs
  actionlib
  pluginlib
  transmission_interface
  urdf
  control_toolbox
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    hardware_interface
    controller_manager
    roscpp
    control_msgs
    trajectory_msgs
    pluginlib
    transmission_interface
    urdf
    control_toolbox
  LIBRARIES 
    robot_hardware
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

add_library(robot_hardware src/robot_hardware_interface.cpp)
target_link_libraries(robot_hardware
	${catkin_LIBRARIES})

add_executable(robot_hardware_interface_node src/robot_hardware_interface_node.cpp)
add_dependencies(robot_hardware_interface_node robot_hardware)

target_link_libraries(robot_hardware_interface_node
	robot_hardware
	${catkin_LIBRARIES})