// common
#include <iostream>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ia_fpcs.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

#include "func.hpp"

int main ()
{ 
    pcl::console::TicToc time;

    auto cloud_source = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_target = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    pcl::io::loadPCDFile("../../PCD/reg_source.pcd", *cloud_source);
    pcl::io::loadPCDFile("../../PCD/reg_target.pcd", *cloud_target);

    voxel_grid_filter<pcl::PointXYZ>(cloud_source, 0.25f);
    voxel_grid_filter<pcl::PointXYZ>(cloud_target, 0.1f);

    time.tic();
    pcl::registration::FPCSInitialAlignment<pcl::PointXYZ, pcl::PointXYZ> fpcs;
    fpcs.setInputSource(cloud_source);
    fpcs.setInputTarget(cloud_target);
    fpcs.setApproxOverlap(0.45);            // 近似重叠率
    fpcs.setDelta(0.1);                     // 精度参数, 配准之后源点云和目标点云之间的距离
    fpcs.setNumberOfThreads(8);             // 线程数
    fpcs.setNumberOfSamples(500);           // 采样点数

    auto align_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    fpcs.align(*align_cloud); 
    if(fpcs.hasConverged()){
        std::cout << "Align source point cloud size is: " << cloud_source->size() << std::endl;
        std::cout << "Align target point cloud size is: " << cloud_target->size() << std::endl;
        std::cout << "Align time is: " << time.toc() << " ms" << std::endl;
        std::cout << "Align score is: "  << fpcs.getFitnessScore() << std::endl;
        std::cout << "Align transformation is: \n" << fpcs.getFinalTransformation() << std::endl;
    } else{
        std::cout << "Align failed!" << std::endl;
    }

    pcl_viewer_init<pcl::PointXYZ, pcl::PointXYZ>(
        cloud_source, "source_before", {1, 0, 0},
        cloud_target, "target_before", {0, 1, 0},
        align_cloud,    "source_after",  {1, 0, },
        cloud_target, "target_after",  {0, 1, 0}
    );

    return 0;
}
