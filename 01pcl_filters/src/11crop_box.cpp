// common
#include <iostream>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "func.hpp"

int main ()
{ 
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    pcl::io::loadPCDFile("../../PCD/rmuc_2025.pcd", *cloud);

    pcl::CropBox<pcl::PointXYZ> box_filter;
    // x y z w   w is 1 usually 
    Eigen::Vector4f min_point(-0.5, -0.5, -0.5, 1); // 立方体左下角坐标
    Eigen::Vector4f max_point(1, 5, 3.4, 1);  //右上角
    box_filter.setMin(min_point);
    box_filter.setMax(max_point);
    box_filter.setInputCloud(cloud);
    box_filter.setKeepOrganized(false);

    auto indices = std::make_shared<pcl::Indices>();
    box_filter.filter(*indices);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    auto cloud_in_box = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    extract.filter(*cloud_in_box);
    auto cloud_out_box = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    extract.setNegative(true);
    extract.filter(*cloud_out_box);

    std::cout << "Cloud points before filtering: " << cloud->points.size() << std::endl;
    std::cout << "Cloud points after filtering: " << cloud_filtered->points.size()<< std::endl;

    pcl_viewer_init<pcl::PointXYZ, pcl::PointXYZ>(cloud_in_box, cloud_out_box);

    return 0;
}
