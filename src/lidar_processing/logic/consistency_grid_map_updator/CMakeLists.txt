project(consistency_grid_map_updator)

add_library(
    ${PROJECT_NAME} SHARED
    ${PROJECT_SOURCE_DIR}/src/consistency_grid_map_updator.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} grid_map lidar_stixel_data_container)