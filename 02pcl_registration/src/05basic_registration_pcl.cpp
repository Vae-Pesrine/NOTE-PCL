#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>

#include <pcl/io/pcd_io.h>

#include "func.hpp"

int main(int argc, char** argv)
{
    auto cloud_source  = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_target  = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    pcl::io::loadPCDFile("../../PCD/reg_source.pcd", *cloud_source);
    pcl::io::loadPCDFile("../../PCD/reg_target.pcd", *cloud_target);

    voxel_grid_filter<pcl::PointXYZ>(cloud_source, 0.25f);
    voxel_grid_filter<pcl::PointXYZ>(cloud_target, 0.1f);

    auto align_time_begin = std::chrono::high_resolution_clock::now();

    // downsample points and convert them into pcl::PointCloud<pcl::Covariance> 
    auto target = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>
                  (*cloud_target, 0.15f);

    auto source = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>
                  (*cloud_source, 0.15f);

    constexpr int num_threads = 4;
    constexpr int num_neighbors = 10;

    // estimate the covariance of the points
    small_gicp::estimate_covariances_omp(*target, num_neighbors, num_threads);
    small_gicp::estimate_covariances_omp(*source, num_neighbors, num_threads); 

    auto target_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>
                       (target, small_gicp::KdTreeBuilderOMP(num_threads));
    auto source_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>
                       (source, small_gicp::KdTreeBuilderOMP(num_threads));

    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration;
    registration.rejector.max_dist_sq = 1.0f;
    registration.reduction.num_threads = num_threads;

    // align point clouds, the input point cloud are pcl::PointCloud<pcl::PointCovariance>
    auto result = registration.align(*target, *source, *target_tree, Eigen::Isometry3d::Identity());

    auto align_time_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> align_time = align_time_end - align_time_begin;
    std::cout << "--- T_target_source ---" << std::endl << result.T_target_source.matrix() << std::endl;
    std::cout << "--- Align time is ---: " << align_time.count() << " ms" << std::endl;
    std::cout << "converged:" << result.converged << std::endl;
    std::cout << "error:" << result.error << std::endl;
    std::cout << "iterations:" << result.iterations << std::endl;
    std::cout << "num_inliers:" << result.num_inliers << std::endl;
    std::cout << "--- H ---" << std::endl << result.H << std::endl;
    std::cout << "--- b ---" << std::endl << result.b.transpose() << std::endl;
    
    // auto result2 = registration.align(*source, *target, *source_tree, Eigen::Isometry3d::Identity());
    // std::cout << "--- T_target_source ---" << std::endl << result2.T_target_source.inverse().matrix() << std::endl;

}