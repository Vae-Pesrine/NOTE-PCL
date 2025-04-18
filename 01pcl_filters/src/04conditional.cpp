// common
#include <iostream>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "viewer.h"

int main ()
{ 
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    pcl::io::loadPCDFile("../../PCD/rmuc_2025.pcd", *cloud);

    pcl::StopWatch time;

    // 定义条件滤波器使用的条件，字段包括，“x” “y” “z” “curvature”
    auto range_cloud = std::make_shared<pcl::ConditionAnd<pcl::PointXYZ>>();
    range_cloud->addComparison(std::make_shared<pcl::FieldComparison<pcl::PointXYZ>>
                                   ("x", pcl::ComparisonOps::GT, 2.0));
    range_cloud->addComparison(std::make_shared<pcl::FieldComparison<pcl::PointXYZ>>
                                   ("x", pcl::ComparisonOps::LT, 7.0)); 


    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cloud);
    condrem.setInputCloud(cloud);
    condrem.setKeepOrganized(true); // 设为true保持点云结构，保存原有点云结构就是点的数目没有改变，采用nan代替
    condrem.filter(*cloud_filtered);


    std::cout << "Cloud points before filtering: " << cloud->points.size() << std::endl;
    std::cout << "Cloud points before removing nan points: " << cloud_filtered->points.size()<< std::endl;
    std::cout << "Time used: " << time.getTime() << " ms" << std::endl;

    // mapping 是非nan点的索引
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, mapping);
    std::cout << "Cloud points after removing nan points: " << cloud_filtered->points.size() << std::endl;

    pcl_viewer_init<pcl::PointXYZ, pcl::PointXYZ>(cloud, cloud_filtered);

    return 0;
}


