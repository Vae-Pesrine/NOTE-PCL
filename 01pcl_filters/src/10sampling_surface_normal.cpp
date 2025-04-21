// common
#include <iostream>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/sampling_surface_normal.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "func.hpp"

int main ()
{ 
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();

    pcl::io::loadPCDFile("../../PCD/rmuc_2025.pcd", *cloud);

    // compute normals
    auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    n.setInputCloud(cloud);
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    n.setSearchMethod(tree);
    n.setNumberOfThreads(8);
    n.setKSearch(30);
    n.compute(*normals);

    // connect the xyz and normal
    auto cloud_with_normals = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    pcl::SamplingSurfaceNormal<pcl::PointNormal> ssn;
    ssn.setInputCloud(cloud_with_normals);
    ssn.setSample(10);
    ssn.setRatio(0.1f);
    ssn.setSeed(0);
    ssn.filter(*cloud_filtered);

    std::cout << "Cloud points before sampling: " << cloud->points.size() << std::endl;
    std::cout << "Cloud points after sampling: " << cloud_filtered->points.size()<< std::endl;

    pcl_viewer_init<pcl::PointXYZ, pcl::PointNormal>(cloud, cloud_filtered);

    return 0;
}
