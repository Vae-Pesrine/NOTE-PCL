// common
#include <iostream>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/model_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "func.hpp"

int main ()
{ 
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    pcl::io::loadPCDFile("../../PCD/rmuc_2025.pcd", *cloud);

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(cloud);
    ror.setRadiusSearch(0.1);
    ror.setMinNeighborsInRadius(10);
    ror.filter(*cloud);

    // 滤波之后加一个立方体点云
    auto cube_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    float cube_min = 8.0f, cube_max = 9.0f; // 立方体范围
    for (float x = cube_min; x <= cube_max; x += 0.1f) {
        for (float y = cube_min; y <= cube_max; y += 0.1f) {
            for (float z = 0.0f; z <= 1.0f; z += 0.1f) {
                cube_cloud->points.emplace_back(x, y, z);
            }
        }
    }
    cube_cloud->width = cube_cloud->points.size();
    cube_cloud->height = 1;
    cube_cloud->is_dense = true;
    *cloud += *cube_cloud;

    pcl::ModelCoefficients coefficients;
    coefficients.values = {8, 8, 0, }; // 设置模型参数
    pcl::ModelOutlierRemoval<pcl::PointXYZ> model_filter;
    model_filter.setInputCloud(cloud);
    model_filter.setModelCoefficients(coefficients);
    model_filter.setModelType(pcl::SACMODEL_SPHERE);
    model_filter.filter(*cloud_filtered);

    pcl_viewer_init<pcl::PointXYZ, pcl::PointXYZ>(cloud, cloud_filtered);

    return 0;
}


