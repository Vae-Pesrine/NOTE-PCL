cmake_minimum_required(VERSION 3.0.2)

project(pcl_filters)

find_package(PCL 1.12 REQUIRED)

include_directories(
    $(CMAKE_CURRENT_SOURCE_DIR)/../pcl_util
)

add_executable(01icp src/01icp.cpp)
target_link_libraries(01icp ${PCL_LIBRARIES})

add_executable(01kd_icp src/01kd_icp.cpp)
target_link_libraries(01kd_icp ${PCL_LIBRARIES})

add_executable(01plane_icp src/01plane_icp.cpp)
target_link_libraries(01plane_icp ${PCL_LIBRARIES})