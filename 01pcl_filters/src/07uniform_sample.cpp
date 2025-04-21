// common
#include <iostream>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "func.hpp"

int main ()
{ 
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    pcl::io::loadPCDFile("../../PCD/rmuc_2025.pcd", *cloud);

    pcl::UniformSampling<pcl::PointXYZ> us;
    us.setInputCloud(cloud);
    us.setRadiusSearch(0.2f);       //正方体体素的边长
    us.filter(*cloud_filtered);
    
    std::cout << "Cloud points before sampling: " << cloud->points.size() << std::endl;
    std::cout << "Cloud points after sampling: " << cloud_filtered->points.size()<< std::endl;

    pcl_viewer_init<pcl::PointXYZ, pcl::PointXYZ>(cloud, cloud_filtered);

    return 0;
}
