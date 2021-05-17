project(lidar_spherical_data_container)

find_package(PCL 1.7 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

find_package(Boost REQUIRED)

add_library(
    ${PROJECT_NAME} SHARED
    ${PROJECT_SOURCE_DIR}/src/lidar_spherical_data_container.cpp)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARY_DIRS} ${Boost_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)