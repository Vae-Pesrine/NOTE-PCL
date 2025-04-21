#pragma once

#include <thread>
#include <chrono>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>

template<typename PointT1, typename PointT2>
void pcl_viewer_init(typename pcl::PointCloud<PointT1>::Ptr &cloud_1,
                     typename pcl::PointCloud<PointT2>::Ptr &cloud_2) {

    auto view = std::make_shared<pcl::visualization::PCLVisualizer>("Show Cloud");
    int v1(0);
    view->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    view->setBackgroundColor(0, 0, 0, v1);
    view->addText("Raw point clouds", 10, 10, "v1_text", v1);
    int v2(0);
    view->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    view->setBackgroundColor(0.1, 0.1, 0.1, v2);
    view->addText("filtered point clouds", 10, 10, "v2_text", v2);

    // 在z字段上进行渲染
    pcl::visualization::PointCloudColorHandlerGenericField<PointT1> f1(cloud_1, "z");
    pcl::visualization::PointCloudColorHandlerGenericField<PointT2> f2(cloud_2, "z");

    view->addPointCloud<PointT1>(cloud_1, f1, "sample cloud", v1);
    view->addPointCloud<PointT2>(cloud_2, f2, "cloud filtered", v2);


    while (!view->wasStopped()){
        view->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

/**
 * @brief the viewer oof pcl registration
 * @param cloud_left the point cloud before registration
 * @param cloud_right the point cloud after registration
 * @param name the name of the point cloud
 * @param color the color of the point cloud  std::vector<double> {r, g, b}
 */
template<typename PointT1, typename PointT2>
void pcl_viewer_init(
    typename pcl::PointCloud<PointT1>::Ptr &cloud1_left,  const std::string &name1_left,  const std::vector<double> &color1_left,
    typename pcl::PointCloud<PointT2>::Ptr &cloud2_left,  const std::string &name2_left,  const std::vector<double> &color2_left,
    typename pcl::PointCloud<PointT1>::Ptr &cloud1_right, const std::string &name1_right, const std::vector<double> &color1_right,
    typename pcl::PointCloud<PointT2>::Ptr &cloud2_right, const std::string &name2_right, const std::vector<double> &color2_right
) {
    auto view = std::make_shared<pcl::visualization::PCLVisualizer>("Show Cloud");
    int v1(0), v2(0);
    view->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    view->setBackgroundColor(0, 0, 0, v1);
    view->addText("Left View", 10, 10, "v1_text", v1);

    view->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    view->setBackgroundColor(0.1, 0.1, 0.1, v2);
    view->addText("Right View", 10, 10, "v2_text", v2);

    // 左侧视口
    view->addPointCloud<PointT1>(cloud1_left, name1_left, v1);
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color1_left[0], color1_left[1], color1_left[2], name1_left, v1);
    view->addPointCloud<PointT2>(cloud2_left, name2_left, v1);
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color2_left[0], color2_left[1], color2_left[2], name2_left, v1);

    // 右侧视口
    view->addPointCloud<PointT1>(cloud1_right, name1_right, v2);
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color1_right[0], color1_right[1], color1_right[2], name1_right, v2);
    view->addPointCloud<PointT2>(cloud2_right, name2_right, v2);
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color2_right[0], color2_right[1], color2_right[2], name2_right, v2);

    while (!view->wasStopped()){
        view->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void voxel_grid_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.1f, 0.1f, 0.1f); 
    vg.filter(*cloud_filtered);
    cloud->clear();
    *cloud = *cloud_filtered;
}

/**
 * @brief compute the normals of point cloud and concatenate to the point cloud
 */
void cloud_with_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_normal)
{
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setNumberOfThreads(4);
    ne.setKSearch(10);
    ne.compute(*normals);
    pcl::concatenateFields(*cloud, *normals, *cloud_normal);
}