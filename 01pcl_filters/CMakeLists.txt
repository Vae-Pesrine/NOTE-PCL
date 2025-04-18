cmake_minimum_required(VERSION 3.0.2)

project(pcl_filters)

find_package(PCL 1.12 REQUIRED)

include_directories(
    $(CMAKE_CURRENT_SOURCE_DIR)/../pcl_util
)

add_executable(01pcl_filter src/01pass_through.cpp)
target_link_libraries(01pcl_filter ${PCL_LIBRARIES})

add_executable(01pcl_filter_advanced src/01pass_through_advanced.cpp)
target_link_libraries(01pcl_filter_advanced ${PCL_LIBRARIES})

add_executable(02voxelgrid src/02voxelgrid.cpp)
target_link_libraries(02voxelgrid ${PCL_LIBRARIES})

add_executable(02voxelgrid_approximate src/02voxelgrid_approximate.cpp)
target_link_libraries(02voxelgrid_approximate ${PCL_LIBRARIES})

add_executable(02voxelgrid_opt src/02voxelgrid_opt.cpp)
target_link_libraries(02voxelgrid_opt ${PCL_LIBRARIES})

add_executable(03radius src/03radius.cpp)
target_link_libraries(03radius ${PCL_LIBRARIES})

add_executable(04conditional src/04conditional.cpp)
target_link_libraries(04conditional ${PCL_LIBRARIES})

add_executable(05model src/05model.cpp)
target_link_libraries(05model ${PCL_LIBRARIES})

add_executable(06indices src/06indices.cpp)
target_link_libraries(06indices ${PCL_LIBRARIES})

add_executable(06indices_advanced src/06indices_advanced.cpp)
target_link_libraries(06indices_advanced ${PCL_LIBRARIES})

add_executable(07uniform_sample src/07uniform_sample.cpp)
target_link_libraries(07uniform_sample ${PCL_LIBRARIES})

add_executable(08random_sample src/08random_sample.cpp)
target_link_libraries(08random_sample ${PCL_LIBRARIES})

add_executable(09normal_space src/09normal_space.cpp)
target_link_libraries(09normal_space ${PCL_LIBRARIES})

add_executable(10sampling_surface_normal src/10sampling_surface_normal.cpp)
target_link_libraries(10sampling_surface_normal ${PCL_LIBRARIES})

add_executable(11crop_box src/11crop_box.cpp)
target_link_libraries(11crop_box ${PCL_LIBRARIES})








