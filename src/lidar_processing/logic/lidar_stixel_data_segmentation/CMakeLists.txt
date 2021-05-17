project(lidar_stixel_data_segmentation)

add_library(
    ${PROJECT_NAME} SHARED
    ${PROJECT_SOURCE_DIR}/src/lidar_stixel_data_segmentation.cpp)
target_link_libraries(${PROJECT_NAME} lidar_stixel_data_container)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)

