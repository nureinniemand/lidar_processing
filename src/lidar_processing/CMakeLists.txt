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

include_directories(ros/include logic/include ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
  INCLUDE_DIRS
  LIBRARIES
  roscpp
  ibeo_processor)

add_library(data_convertor ros/src/data_convertor/ibeo_points_convertor.cpp)
target_link_libraries(data_convertor ${catkin_LIBRARIES} stixel_cylindrical_data_container)

add_executable(ibeo_processor ros/src/application/main.cpp ros/src/application/ibeo_processor.cpp )
target_link_libraries(ibeo_processor ${catkin_LIBRARIES} data_convertor)



