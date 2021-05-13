project(lidar_processing_logic)

find_package(PCL 1.7 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

find_package(Boost REQUIRED)

add_library(
    lidar_spherical_data_container SHARED
    ${PROJECT_SOURCE_DIR}/src/lidar_spherical_data_container/lidar_spherical_data_container.cpp)
target_link_libraries(lidar_spherical_data_container ${PCL_LIBRARY_DIRS} ${Boost_LIBRARIES})
target_include_directories(lidar_spherical_data_container PUBLIC ${PCL_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)

add_library(
    stixel_cylindrical_data_container SHARED
    ${PROJECT_SOURCE_DIR}/src/stixel_cylindrical_data_container/stixel_cylindrical_data_container.cpp)
target_link_libraries(stixel_cylindrical_data_container ${PCL_LIBRARY_DIRS} ${Boost_LIBRARIES})
target_include_directories(stixel_cylindrical_data_container PUBLIC ${PCL_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)
