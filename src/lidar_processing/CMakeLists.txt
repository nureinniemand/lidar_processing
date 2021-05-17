cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(lidar_processing)

set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -Wall -Wextra -Werror")

add_subdirectory(logic)

find_package(
    catkin REQUIRED
    COMPONENTS roscpp
               pcl_conversions
               pcl_ros
               sensor_msgs
)   

catkin_package(
  CATKIN_DEPENDS
  INCLUDE_DIRS
  LIBRARIES
  roscpp
  pointcloud_segmentation)

add_subdirectory(ros)
