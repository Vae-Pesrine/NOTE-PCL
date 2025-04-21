// common
#include <iostream>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/normal_space.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "func.hpp"

int main ()
{ 
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    pcl::io::loadPCDFile("../../PCD/rmuc_2025.pcd", *cloud);

    auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    n.setInputCloud(cloud);
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    n.setSearchMethod(tree);
    n.setNumberOfThreads(8);
    n.setKSearch(30);
    n.compute(*normals);

    pcl::NormalSpaceSampling<pcl::PointXYZ, pcl::Normal> ns;
    ns.setInputCloud(cloud);
    ns.setNormals(normals);
    // 法线空间在每个方向上划分为2个bin，总共8个格子，每个点的法线会被分配到某一个bin中
    ns.setBins(2, 2, 2);
    ns.setSeed(0);
    ns.setSample(80000);
    ns.filter(*cloud_filtered);

    std::cout << "Cloud points before sampling: " << cloud->points.size() << std::endl;
    std::cout << "Cloud points after sampling: " << cloud_filtered->points.size()<< std::endl;

    pcl_viewer_init<pcl::PointXYZ, pcl::PointXYZ>(cloud, cloud_filtered);

    return 0;
}
