project(grid_map)

add_library(
    ${PROJECT_NAME} SHARED
    ${PROJECT_SOURCE_DIR}/src/grid_map.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)