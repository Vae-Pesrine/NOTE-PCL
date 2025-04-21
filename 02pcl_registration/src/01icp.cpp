// common
#include <iostream>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
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

    voxel_grid_filter(cloud_source);
    voxel_grid_filter(cloud_target);

    time.tic();
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target);
    icp.setTransformationEpsilon(1e-10);
    icp.setMaximumIterations(50);
    icp.setUseReciprocalCorrespondences(true);

    auto icp_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    icp.align(*icp_cloud);
    
    if(icp.hasConverged()){
        std::cout << "Align time is: " << time.toc() << " ms" << std::endl;
        std::cout << "Align score is: "  << icp.getFitnessScore() << std::endl;
        std::cout << "Align transformation is: " << icp.getFinalTransformation() << std::endl;
    } else{
        std::cout << "Align failed!" << std::endl;
    }

    pcl_viewer_init<pcl::PointXYZ, pcl::PointXYZ>(
        cloud_source, "source_before", {1, 0, 0},
        cloud_target, "target_before", {0, 1, 0},
        icp_cloud, "source_after", {0, 0, 1},
        cloud_target, "target_after", {0, 1, 0}
    );

    return 0;
}
