#include <iostream>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

int main()
{
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->is_dense = false;
    // 生成点云 
    pcl::PointXYZ p;
    for (size_t i = 0; i < 5; ++i){
        p.x = p.y = p.z = static_cast<float>(i);
        cloud->push_back(p);
    }

    std::cout << "Cloud has " << cloud->points.size() << " points." << std::endl;

    //-----------根据索引提取点云------------------
    // 1、取得需要的索引，提取点云中第0和第2个点
    pcl::PointIndices indices;
    indices.indices.push_back(0);
    indices.indices.push_back(2);
    // 2、索引提取器
    pcl::ExtractIndices<pcl::PointXYZ> extr;
    extr.setInputCloud(cloud);//设置输入点云
    extr.setIndices(std::make_shared<const pcl::PointIndices>(indices));//设置索引
    auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    extr.filter(*output);     //提取对应索引的点云
    
    std::cout << "Output has " << output->points.size() << " points." << std::endl;
    auto cloud_other = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    // 3、提取对应索引之外的点    
    extr.setNegative(true); 
    extr.filter(*cloud_other);

    std::cout << "Other has " << cloud_other->points.size() << " points." << std::endl;

    return (0);
}

