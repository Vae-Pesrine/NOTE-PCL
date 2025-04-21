// common
#include <iostream>
#include <vector>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "func.hpp"

int main ()
{
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    pcl::io::loadPCDFile("../../PCD/rmuc_2025.pcd", *cloud);
    
    // 创建滤波器对象
    pcl::PassThrough<pcl::PointXYZ> pass(true);  // 设置为true允许我们提取被删除的索引
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");          
    pass.setFilterLimits(2.0, 7.0);        
    pass.setNegative(true);                // 删除给定范围内的点
    std::vector<int> indices;
    pass.filter(indices); // 获取被删除的点的索引

    pcl::IndicesConstPtr indices_removed = pass.getRemovedIndices(); // 获取除indices外其他点的索引

    auto index_x = std::make_shared<std::vector<int>>(indices);
    pass.setIndices(index_x); // 设置要删除的点的索引
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);
    pass.setNegative(true); // 删除给定范围内的点
    pass.filter(*cloud_filtered);

    std::cout << "Cloud points before filtering: " << cloud->points.size() << std::endl;
    std::cout << "Cloud points after filtering: " << cloud_filtered->points.size()<< std::endl;
    
    pcl_viewer_init<pcl::PointXYZ, pcl::PointXYZ>(cloud, cloud_filtered);

    return 0;
}
