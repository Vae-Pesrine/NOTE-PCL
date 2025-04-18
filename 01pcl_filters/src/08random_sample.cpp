// common
#include <iostream>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/random_sample.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "viewer.h"

int main ()
{ 
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    pcl::io::loadPCDFile("../../PCD/rmuc_2025.pcd", *cloud);

    pcl::RandomSample<pcl::PointXYZ> rs;
    rs.setInputCloud(cloud);
    rs.setSample(15000); // 采样点数
    rs.filter(*cloud_filtered);
    
    std::cout << "Cloud points before sampling: " << cloud->points.size() << std::endl;
    std::cout << "Cloud points after sampling: " << cloud_filtered->points.size()<< std::endl;

    pcl_viewer_init<pcl::PointXYZ, pcl::PointXYZ>(cloud, cloud_filtered);

    return 0;
}
