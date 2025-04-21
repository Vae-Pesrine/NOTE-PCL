// common
#include <iostream>

// pcl
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "func.hpp"

int main(int argc, char** argv)
{
    auto cloud_load = std::make_shared<pcl::PCLPointCloud2>();
    auto cloud_filtered_blob = std::make_shared<pcl::PCLPointCloud2>();
    auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_p = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_f = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_planes = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    pcl::PCDReader reader;
    reader.read("../../PCD/rmuc_2025.pcd", *cloud_load);

    // 下采样
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_load);
    sor.setLeafSize(0.04f, 0.04f, 0.01f);
    sor.filter(*cloud_filtered_blob);

    pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

    // 平面分割
    auto coefficients = std::make_shared<pcl::ModelCoefficients>();
    auto inliers = std::make_shared<pcl::PointIndices>();
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    int nr_points = static_cast<int>(cloud_filtered->points.size());

    // 分割多个平面并合并到cloud_planes
    while (cloud_filtered->points.size() > 0.3 * nr_points)
    {
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.empty())
            break;

        // 提取平面内点
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);

        // 合并到cloud_planes
        *cloud_planes += *cloud_p;

        // 提取平面外点
        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud_filtered.swap(cloud_f);
    }

    // 用模板化可视化函数显示
    pcl_viewer_init<pcl::PointXYZ, pcl::PointXYZ>(cloud_planes, cloud_filtered);

    return 0;
}