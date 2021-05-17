add_executable(pointcloud_segmentation src/main.cpp src/pointcloud_segmentation.cpp )
target_include_directories(pointcloud_segmentation PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(pointcloud_segmentation ${catkin_LIBRARIES} data_convertor)
