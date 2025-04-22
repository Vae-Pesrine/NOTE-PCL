cmake_minimum_required(VERSION 3.0.2)

project(pcl_filters)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.12 REQUIRED)

find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
if(OpenMP_FOUND)
  message(STATUS "OpenMP found: ${OpenMP_CXX_FLAGS}")
else()
  message(FATAL_ERROR "OpenMP not found!")
endif()


include_directories(
    $(CMAKE_CURRENT_SOURCE_DIR)/../pcl_util
)

add_executable(01icp src/01icp.cpp)
target_link_libraries(01icp ${PCL_LIBRARIES})

add_executable(01kd_icp src/01kd_icp.cpp)
target_link_libraries(01kd_icp ${PCL_LIBRARIES})

add_executable(01plane_icp src/01plane_icp.cpp)
target_link_libraries(01plane_icp ${PCL_LIBRARIES})

add_executable(02kd_gicp src/02kd_gicp.cpp)
target_link_libraries(02kd_gicp ${PCL_LIBRARIES})

add_executable(03sac_ia src/03sac_ia.cpp)
target_link_libraries(03sac_ia ${PCL_LIBRARIES})

add_executable(03sac_pj src/03sac_pj.cpp)
target_link_libraries(03sac_pj ${PCL_LIBRARIES})

add_executable(04fpcs src/04fpcs.cpp)
target_link_libraries(04fpcs ${PCL_LIBRARIES})

add_executable(04kfpcs src/04kfpcs.cpp)
target_link_libraries(04kfpcs ${PCL_LIBRARIES})

add_executable(05basic_registration_pcl src/05basic_registration_pcl.cpp)
target_link_libraries(05basic_registration_pcl ${PCL_LIBRARIES} ${OpenMP_CXX_FLAGS})