// common
#include <iostream>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

#include "func.hpp"

pcl::PointCloud<pcl::FPFHSignature33>::Ptr compute_fpfh_feature(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const pcl::search::KdTree<pcl::PointXYZ>::Ptr &tree
) {
    // compute normals
    auto fpfh = std::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_est;
    normal_est.setInputCloud(cloud);
    normal_est.setNumberOfThreads(8);
    normal_est.setSearchMethod(tree);
    normal_est.setKSearch(10);
    auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
    normal_est.compute(*normals);

    // compute fpfh features
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud(cloud);
    fpfh_est.setInputNormals(normals);
    fpfh_est.setSearchMethod(tree);
    fpfh_est.setNumberOfThreads(8);
    fpfh_est.setKSearch(10);
    fpfh_est.compute(*fpfh);

    return fpfh;
}

int main ()
{ 
    pcl::console::TicToc time;

    auto cloud_source = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_target = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    pcl::io::loadPCDFile("../../PCD/reg_source.pcd", *cloud_source);
    pcl::io::loadPCDFile("../../PCD/reg_target.pcd", *cloud_target);

    voxel_grid_filter<pcl::PointXYZ>(cloud_source, 0.25f);
    voxel_grid_filter<pcl::PointXYZ>(cloud_target, 0.1f);

    auto source_fpfh = compute_fpfh_feature(cloud_source, std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>());
    auto target_fpfh = compute_fpfh_feature(cloud_target, std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>());

    time.tic();
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();

    pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_pj;
    sac_pj.setInputSource(cloud_source);
    sac_pj.setSourceFeatures(source_fpfh);
    sac_pj.setInputTarget(cloud_target);
    sac_pj.setTargetFeatures(target_fpfh);
    sac_pj.setCorrespondenceRandomness(10);          // 选择随机特征对应时， 设置要使用的邻居的数量， 数值越大， 特征匹配的随机性越大
    sac_pj.setInlierFraction(0.5);                   // 设置内点的比例， 该值越大， 内点的数量越多
    sac_pj.setNumberOfSamples(10);                   // 设置随机采样的数量， 该值越大， 计算的内点数量越多
    sac_pj.setSimilarityThreshold(0.1f);             // 设置相似性阈值， 该值越大， 内点的数量越多
    sac_pj.setMaxCorrespondenceDistance(2.50f);      // 设置最大对应点距离， 该值越大， 内点的数量越多
    sac_pj.setMaximumIterations(100);                // 设置最大迭代次数， 该值越大， 内点的数量越多


    // no initialguess transformation
    auto icp_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    sac_pj.align(*icp_cloud); 

    if(sac_pj.hasConverged()){
        std::cout << "Align source point cloud size is: " << cloud_source->size() << std::endl;
        std::cout << "Align target point cloud size is: " << cloud_target->size() << std::endl;
        std::cout << "Align time is: " << time.toc() << " ms" << std::endl;
        std::cout << "Align score is: "  << sac_pj.getFitnessScore() << std::endl;
        std::cout << "Align transformation is: \n" << sac_pj.getFinalTransformation() << std::endl;
    } else{
        std::cout << "Align failed!" << std::endl;
    }

    pcl_viewer_init<pcl::PointXYZ, pcl::PointXYZ>(
        cloud_source, "source_before", {1, 0, 0},
        cloud_target, "target_before", {0, 1, 0},
        icp_cloud,    "source_after",  {1, 0, 0},
        cloud_target, "target_after",  {0, 1, 0}
    );

    return 0;
}
