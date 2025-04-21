// common
#include <iostream>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

#include "func.hpp"

int main ()
{ 
    pcl::console::TicToc time;

    auto cloud_source = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_target = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    pcl::io::loadPCDFile("../../PCD/reg_source.pcd", *cloud_source);
    pcl::io::loadPCDFile("../../PCD/reg_target.pcd", *cloud_target);

    voxel_grid_filter(cloud_source);
    voxel_grid_filter(cloud_target);

    auto source_normals = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    auto target_normals = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    cloud_with_normal(cloud_source, source_normals);
    cloud_with_normal(cloud_target, target_normals);

    time.tic();
    pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;

    // KD树加速搜索
    auto tree_source = std::make_shared<pcl::search::KdTree<pcl::PointNormal>>();
    auto tree_target = std::make_shared<pcl::search::KdTree<pcl::PointNormal>>();
    tree_source->setInputCloud(source_normals);
    tree_target->setInputCloud(target_normals);
    icp.setSearchMethodSource(tree_source);
    icp.setSearchMethodTarget(tree_target);

    // icp核心部分
    icp.setInputSource(source_normals);         // 源点云
    icp.setInputTarget(target_normals);         // 目标点云
    icp.setTransformationEpsilon(1e-10);       // 最小转换差异  
    icp.setMaxCorrespondenceDistance(10.0);    // 最大对应点距离
    icp.setEuclideanFitnessEpsilon(0.001);     // 收敛条件是均方差的和小于阈值，停止迭代
    icp.setMaximumIterations(50);             // 最大迭代次数
    /**
     * @brief true是双向匹配，false是单向匹配
     * @details 双向匹配是指在计算对应点时，源点云中的每个点都要在目标点云中找到一个最近邻点，
     * 并且目标点云中的每个点也要在源点云中找到一个最近邻点。这样可以避免单向匹配可能导致的错误匹配。
     * @details 单向匹配是指只在源点云中找到最近邻点，而不考虑目标点云中的点。这样可能会导致一些错误匹配，
     * 因为源点云中的点可能会与目标点云中的远离的点匹配。
     */
    icp.setUseReciprocalCorrespondences(true); 

    auto icp_cloud = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    icp.align(*icp_cloud);
    
    if(icp.hasConverged()){
        std::cout << "Align time is: " << time.toc() << " ms" << std::endl;
        std::cout << "Align score is: "  << icp.getFitnessScore() << std::endl;
        std::cout << "Align transformation is: " << icp.getFinalTransformation() << std::endl;
    } else{
        std::cout << "Align failed!" << std::endl;
    }

    pcl_viewer_init<pcl::PointNormal, pcl::PointNormal>(
        source_normals, "source_before", {1, 0, 0},
        target_normals, "target_before", {0, 1, 0},
        icp_cloud, "source_after", {0, 0, 1},
        target_normals, "target_after", {0, 1, 0}
    );

    return 0;
}
