// common
#include <iostream>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "func.hpp"

int main ()
{ 
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    pcl::io::loadPCDFile("../../PCD/rmuc_2025.pcd", *cloud);

    // 创建滤波器对象
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> avg;
    avg.setInputCloud(cloud);
    avg.setLeafSize(0.25f, 0.25f, 0.25f); // 设置体素大小
    avg.filter(*cloud_filtered); // 过滤点云

    std::cout << "Cloud points before filtering: " << cloud->points.size() << std::endl;
    std::cout << "Cloud points after filtering: " << cloud_filtered->points.size()<< std::endl;

    pcl_viewer_init<pcl::PointXYZ, pcl::PointXYZ>(cloud, cloud_filtered);
    
    return 0;
}


