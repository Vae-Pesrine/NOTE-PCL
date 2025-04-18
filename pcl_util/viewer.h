#pragma once

#include <thread>
#include <chrono>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

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