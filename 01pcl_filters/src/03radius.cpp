// common
#include <iostream>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "func.hpp"

int main ()
{ 
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    pcl::io::loadPCDFile("../../PCD/rmuc_2025.pcd", *cloud);

    // 创建滤波器对象
    pcl::StopWatch time;
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(cloud);
    ror.setRadiusSearch(0.1);
    ror.setMinNeighborsInRadius(10);
    ror.filter(*cloud_filtered);
    pcl::io::savePCDFileBinary("../../PCD/registration.pcd", *cloud_filtered);

    std::cout << "Cloud points before filtering: " << cloud->points.size() << std::endl;
    std::cout << "Cloud points after filtering: " << cloud_filtered->points.size()<< std::endl;
    std::cout << "Time used: " << time.getTime() << " ms" << std::endl;

    pcl_viewer_init<pcl::PointXYZ, pcl::PointXYZ>(cloud, cloud_filtered);

    return 0;
}


