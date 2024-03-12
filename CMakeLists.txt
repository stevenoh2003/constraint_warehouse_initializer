cmake_minimum_required(VERSION 3.0.2)
project(warehouse_database)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_warehouse
  warehouse_ros
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS moveit_core moveit_ros_planning moveit_ros_warehouse warehouse_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(initialize_demo_db src/initialize_demo_db.cpp)


target_link_libraries(initialize_demo_db ${catkin_LIBRARIES})

add_executable(set_constraint src/set_constraint.cpp src/db_initializer.cpp)
target_link_libraries(set_constraint ${catkin_LIBRARIES})