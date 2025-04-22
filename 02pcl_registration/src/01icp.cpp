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

    voxel_grid_filter<pcl::PointXYZ>(cloud_source, 0.25f);
    voxel_grid_filter<pcl::PointXYZ>(cloud_target, 0.1f);

    time.tic();
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target);
    icp.setTransformationEpsilon(1e-6);
    icp.setMaximumIterations(50);
    icp.setUseReciprocalCorrespondences(true);

    Eigen::Matrix4f init_transform = Eigen::Matrix4f::Identity();
    float x = 2.0f, y = 1.5f, z = 1.0f; 
    init_transform(0, 3) = x;
    init_transform(1, 3) = y;
    init_transform(2, 3) = z;
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f; // rad
    Eigen::Matrix3f rotation_matrix = (Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                                       Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                                       Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())).toRotationMatrix();
    init_transform.block<3, 3>(0, 0) = rotation_matrix;
   
    auto icp_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    icp.align(*icp_cloud, init_transform); 
    if(icp.hasConverged()){
        std::cout << "Align source point cloud size is: " << cloud_source->size() << std::endl;
        std::cout << "Align target point cloud size is: " << cloud_target->size() << std::endl;
        std::cout << "Align time is: " << time.toc() << " ms" << std::endl;
        std::cout << "Align score is: "  << icp.getFitnessScore() << std::endl;
        std::cout << "Align transformation is: \n" << icp.getFinalTransformation() << std::endl;
    } else{
        std::cout << "Align failed!" << std::endl;
    }

    pcl_viewer_init<pcl::PointXYZ, pcl::PointXYZ>(
        cloud_source, "source_before", {1, 0, 0},
        cloud_target, "target_before", {0, 1, 0},
        icp_cloud,    "source_after",  {1, 0, },
        cloud_target, "target_after",  {0, 1, 0}
    );

    return 0;
}
