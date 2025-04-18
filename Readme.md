# <center>**Point Cloud Libraries**

## make and run
```SHELL
cd 01pcl_filters/build
cmake ..
make     # build target executable cpp
./01pass_through # run 
```

## todo
- [x] Add filter methods
- [ ] Add registration methods
- [ ] Add small gicp methods
- [ ] Add segmentation methods
- [ ] Add transform for pcl
- [ ] Add keypoints extract
- [ ] Add feature descriptor
- [ ] Add a script to automatically change the version of pcl


## **pcl filters**

### 1.pass through filter 
&emsp;&emsp;直通滤波，过滤掉在指定维度上取值不在给定值域内的.pcl::PassThrough 类实现对用户给定点云某个字段的限定下，对点云进行简单的基本过滤，例如限制过滤掉点云中所有z字段不在某个范围内的点, 示例代码 [01pass_through.cpp](01pcl_filters/src/01pass_through.cpp)

主要函数：

```c++
void setFilterFieldName(const std::string &field_name)
```    
设置限定字段的字符串名称，例如 “x”， “y”， “z”， “intensity”

```c++
void setFilterLimits (const double &limit_min, const double &limit_max)
```
设置限定滤波条件，包括最小值limit_min和最大值limit_max, 点云中所有在设定范围内的点被删除

```c++
void setNegative (bool &negative):
```
设置为true，返回范围外的点，false则返回范围内的点


```c++
pcl::PassThrough<pcl::PointXYZ> pass(true); // 用true初始化将允许我们提取被删除的索引
```
获取点的索引用法详细见示例代码 [01pass_through_advanced.cpp](01pcl_filters/src/01pass_through_advanced.cpp)

### 2. voxel filter
&emsp;&emsp;点云数量较多时，对后面的分割工作等会带来困难，所以对其进行降采样（下采样），使用体素网格化的方式实现，可以减少点的数量又能保存点云的形状特征，可以提高后续算法的速度， 示例代码 [02voxel_grid.cpp]
(01pcl_filters/src/02voxelgrid.cpp)

&emsp;&emsp;pcl::VoxelGrid 类是通过输入的点云数据创建一个三维体素栅格，每体素内用体素中所有点的重心来近似地显示体素中其他点，这样该体素内所有点都用一个重心点来表示，对于所有体素进行处理后得到过滤后的点云，比用体素中心（注意中心和重心）逼近的方法更慢，但是对于采样点对应曲面的表示更为准确。体素滤波容易造成特征的丢失，适用于均匀分布点云的精简 示例代码
[02voxelgrid.cpp](01pcl_filters/src/02voxelgrid.cpp)

&emsp;&emsp;pcl：：ApproximateVoxelGrid是通过输入的点云数据创建一个三维体素栅格，用每个体素内所有点的中心来近似表示体素中的其他点，这种计算方法基于哈希函数完成 示例代码 [02voxelgrid_approximate.cpp](01pcl_filters/src/02voxelgrid_approximate.cpp)

&emsp;&emsp;改进的体素滤波，使用pcl::VoxelGrid滤波之后，计算近邻点，使用原始点云数据中距离体素重心最近的点代替体素中心点，提高点云数据的表达准确性 示例代码 [02voxelgrid_opt.cpp](01pcl_filters/src/02voxelgrid_opt.cpp)

主要函数：
```c++
setInputCloud (const PCLPointCloud2ConstPtr &cloud);
```
设置待滤波的点云
```c++
 void  setLeafSize (const Eigen::Vector3f &leaf_size)
```
设置每个方向上栅格的尺寸大小

### 3. radius filter
&emsp;&emsp;在点云数据中，设定每个点一定半径范围内周围至少有足够多的近邻，不满足就会被删除。流程：点云数据构造kdtree，建立点云拓扑关系求点云中任意一点邻域范围内近邻点个数;判断近邻点个数是否小于判定阈值，若小于则认为该点为噪声点并去除;重复上述步骤直至suuoyou点云处理完毕。 示例代码[03radius.cpp](01pcl_filters/src/03radius.cpp)

主要函数： 
```c++
setInputCloud (const PCLPointCloud2ConstPtr &cloud)
```
设置待滤波的点云
```c++
setRadiusSearch(double radius)
```
设置半径范围
```c++
setMinNeighborsInRadius (int min_pts)
```
设置半径范围内邻域点集数小于min_pts的删除


### 4. condition filter
&emsp;&emsp;条件滤波器，可以一次行善除满足对输入点云设定的一个或多个条件指标的所有的数据点。

```c++
auto range_cloud = std::make_shared<pcl::ConditionAnd<pcl::PointXYZ>>();
range_cloud->addComparison(std::make_shared<pcl::FieldComparison<pcl::PointXYZ>>
                               ("x", pcl::ComparisonOps::GT, 2.0));
range_cloud->addComparison(std::make_shared<pcl::FieldComparison<pcl::PointXYZ>>
                               ("x", pcl::ComparisonOps::LT, 7.0)); 
```
定义条件滤波使用的条件，可选择的字段有“x”， “y”， “z”， “curvature”
```c++
pcl::ConditionalRemoval<pcl::PointXYZ> removal;
removal.setCondition(range_cloud);
removal.setInputCloud(cloud)
```
使用条件滤波器

### 5. indices extract
索引提取, 示例代码:[06indices.cpp](01pcl_filters/src/06indices.cpp)  [06indices_advanced.cpp](01pcl_filters/src/06indices_advanced.cpp)

### 6. uniform sampling
均匀采样，在输入点云数据上创建一个3d体素网络，计算每个网络重心，再使用k近邻搜索寻找距离它最近的点来代表重心点。
示例代码 [07uniform_sample.cpp](01pcl_filters/src/07uniform_sample.cpp)
### 7. random sampling
&emsp;&emsp;随机采样，通过生成随机数种子对点云索引进行筛选从而获得精简点云的方法。但该算法不是按照空间拓扑关系进行的，所以采样后数据在空间上会产生密度分布不均匀的效果。
&emsp;&emsp;随机降采样通过采样率作为阈值控制点云的采样效率。设原始点云输入数量为 N+1， 其对应点云索引为0,1,2,...,N，采样率为K，则算法通过生成（N+1）× K个随机种子。利用种子进行采样
示例代码 [08random_sample.cpp](01pcl_filters/src/08random_sample.cpp)
### 8. normal space sampling
&emsp;&emsp;法线空间采样，在法向量空间中均匀随机抽样，使所选点之间的法线分布尽可能大，结果表现为地表特征变化大的地方剩余点较多，变化小的地方剩余点稀少。
示例代码 [09normal_space.cpp](01pcl_filters/src/08random_sample.cpp)
### 9. sampling surface normal
&emsp;&emsp;索引空间采样，将输入空间划分为网格，直到每个网格中最多包含N个点，并在每个网格中随机采样点，使用每个网格的N个点计算法向量，网格内所有点都被赋予相同的法向量。每个网格中的采样点数为 最大样本数(setSample())×采样率(setRatio())。
示例代码 [10sampling_surface_normal.cpp](01pcl_filters/src/10sampling_surface_normal.cpp)
### 10. get all points at the specified elevation 
```c++
auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

float height = 10;           // 指定高程 
float precision = 0.01;      // 精度
for(const auto &point : cloud){
    if((point->z >= height - precision) && (point->z <= height + precision)){
        cloud_filtered->push_back(point);
    }

}
```

### 11. crop box
设置一个立方体，对立方体内的点云进行裁剪 
示例代码 [11crop_box.cpp](01pcl_filters/src//11crop_box.cpp)









