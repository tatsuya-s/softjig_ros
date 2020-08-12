cmake_minimum_required(VERSION 2.8.3)
project(softjig_ros)

add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    rospy
    std_msgs
    control_msgs
    actionlib_msgs
    tf2_ros
    pcl_ros
    iiwa_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    rospy
    std_msgs
    control_msgs
    actionlib_msgs
    tf2_ros
    pcl_ros
    iiwa_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  softjig_controller_node
    src/softjig_controller_node.cpp
)

add_dependencies(
  softjig_controller_node
    iiwa_msgs_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  softjig_controller_node
    ${catkin_LIBRARIES}
)