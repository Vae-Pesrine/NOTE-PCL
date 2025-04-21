# <center>**Point Cloud Libraries**

## Make and run
```SHELL
# example goes for others
cd 01pcl_filters/build
cmake ..
make     # build target executable cpp
./01pass_through # run 
```

## Todo
- [x] Add filter methods
- [ ] Add registration methods
- [ ] Add small gicp methods
- [ ] Add segmentation methods
- [ ] Add transform for pcl
- [ ] Add keypoints extract
- [ ] Add feature descriptor
- [ ] Add a script to automatically change the version of pcl


## **Point cloud filter**

### 1.pass through filter 
&emsp;&emsp;直通滤波，过滤掉在指定维度上取值不在给定值域内的.pcl::PassThrough 类实现对用户给定点云某个字段的限定下，对点云进行简单的基本过滤，例如限制过滤掉点云中所有z字段不在某个范围内的点, Code [01pass_through.cpp](01pcl_filters/src/01pass_through.cpp)

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
Code [01pass_through_advanced.cpp](01pcl_filters/src/01pass_through_advanced.cpp)

### 2. voxel filter
&emsp;&emsp;点云数量较多时，对后面的分割工作等会带来困难，所以对其进行降采样（下采样），使用体素网格化的方式实现，可以减少点的数量又能保存点云的形状特征，可以提高后续算法的速度， Code [02voxel_grid.cpp]
(01pcl_filters/src/02voxelgrid.cpp)

&emsp;&emsp;pcl::VoxelGrid 类是通过输入的点云数据创建一个三维体素栅格，每体素内用体素中所有点的重心来近似地显示体素中其他点，这样该体素内所有点都用一个重心点来表示，对于所有体素进行处理后得到过滤后的点云，比用体素中心（注意中心和重心）逼近的方法更慢，但是对于采样点对应曲面的表示更为准确。体素滤波容易造成特征的丢失，适用于均匀分布点云的精简 Code
[02voxelgrid.cpp](01pcl_filters/src/02voxelgrid.cpp)

&emsp;&emsp;pcl：：ApproximateVoxelGrid是通过输入的点云数据创建一个三维体素栅格，用每个体素内所有点的中心来近似表示体素中的其他点，这种计算方法基于哈希函数完成 Code [02voxelgrid_approximate.cpp](01pcl_filters/src/02voxelgrid_approximate.cpp)

&emsp;&emsp;改进的体素滤波，使用pcl::VoxelGrid滤波之后，计算近邻点，使用原始点云数据中距离体素重心最近的点代替体素中心点，提高点云数据的表达准确性 Code [02voxelgrid_opt.cpp](01pcl_filters/src/02voxelgrid_opt.cpp)

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
&emsp;&emsp;在点云数据中，设定每个点一定半径范围内周围至少有足够多的近邻，不满足就会被删除。流程：点云数据构造kdtree，建立点云拓扑关系求点云中任意一点邻域范围内近邻点个数;判断近邻点个数是否小于判定阈值，若小于则认为该点为噪声点并去除;重复上述步骤直至suuoyou点云处理完毕。 Code [03radius.cpp](01pcl_filters/src/03radius.cpp)

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
索引提取, Code :[06indices.cpp](01pcl_filters/src/06indices.cpp)  [06indices_advanced.cpp](01pcl_filters/src/06indices_advanced.cpp)

### 6. uniform sampling
均匀采样，在输入点云数据上创建一个3d体素网络，计算每个网络重心，再使用k近邻搜索寻找距离它最近的点来代表重心点。
Code [07uniform_sample.cpp](01pcl_filters/src/07uniform_sample.cpp)
### 7. random sampling
&emsp;&emsp;随机采样，通过生成随机数种子对点云索引进行筛选从而获得精简点云的方法。但该算法不是按照空间拓扑关系进行的，所以采样后数据在空间上会产生密度分布不均匀的效果。
&emsp;&emsp;随机降采样通过采样率作为阈值控制点云的采样效率。设原始点云输入数量为 N+1， 其对应点云索引为0,1,2,...,N，采样率为K，则算法通过生成（N+1）× K个随机种子。利用种子进行采样
Code [08random_sample.cpp](01pcl_filters/src/08random_sample.cpp)
### 8. normal space sampling
&emsp;&emsp;法线空间采样，在法向量空间中均匀随机抽样，使所选点之间的法线分布尽可能大，结果表现为地表特征变化大的地方剩余点较多，变化小的地方剩余点稀少。
Code [09normal_space.cpp](01pcl_filters/src/08random_sample.cpp)
### 9. sampling surface normal
&emsp;&emsp;索引空间采样，将输入空间划分为网格，直到每个网格中最多包含N个点，并在每个网格中随机采样点，使用每个网格的N个点计算法向量，网格内所有点都被赋予相同的法向量。每个网格中的采样点数为 最大样本数(setSample())×采样率(setRatio())。
Code [10sampling_surface_normal.cpp](01pcl_filters/src/10sampling_surface_normal.cpp)
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

### 11. Crop box
设置一个立方体，对立方体内的点云进行裁剪 
Code [11crop_box.cpp](01pcl_filters/src//11crop_box.cpp)


## **Point cloud registration**
- I used English for the convenience of writing formulas.

### 1. ICP registration
- Registration process: ICP iteratively corrects the rigid transformation(translation, rotation, etc.) between two original point clouds to minimize the distance between all point sets.
  - Input: Two frames od raw point cloud, initial estimation od transformation, and the criterion for stopping iteration.
  - Output: Transformation matrix and the corrected point cloud after transformation. 
  
- ICP algorithm steps：
  - Apply the initial transformation $T_0$ to each point $P_{ai}$ in the point set $A$ to obtain $P_{ai'}$ 
  - Find the point $P_{bi}$ that is closest to point $P_{ai'}$ from point set $B$ to fform the corresponding point pair
  - Solve for the optimal transformation $\Delta T$ 
  
  $$\Delta T = arg \min_{R, t} \sum_{i=1}^{n} ||P_{bi} - (RP_{ai'}+t)||^2$$

- Detemine whether it coverages based on the errors of the two iterations, the number of iterations, and the conditons, etc. If it coverages, output the final result $T=\Delta T * T_0$; otherwise, $T_0=\Delta T * T_0$. Then repeat the firse step. 

- [Algorithm derivation](https://yilingui.xyz/2019/11/20/191120_point_cloud_registration_icp/)

- Code：[01icp.cpp](02pcl_registration/src/01icp.cpp)

- KD-tree optimization:Construct a KD-tree to implement the nearest neighbor algorithm and conduce nearest neighbor search on the points after coarse registration. The Euclidean distance is used as a judgement standard, the registration key points with the Euckidean larger than threshold value are eliminated, and the points with high registration precision are saved.(Just understand [KD-tree theory](https://www.cnblogs.com/eyeszjwang/articles/2429382.html))

- Bidirectional KD-tree optimized ICP
  - a. Respectively constructing KD-trees of $P$ and $Q$ point cloud.
  - b. Search the nearest point $Q_i$ of $P_i$ in the point set $Q$.
  - c. If the closest point of $Q_i$ is $P_i$ within point set $P$, so $P_i$ and $Q_i$ is a point pair. Otherwise, goes for $Q_{i+1}$.
  - Traverse the points in the point set $P$.
  - Code: [01kd_icp.cpp](02pcl_registration/src/01kd_icp.cpp)
- Point-To-Plane ICP: The traditional ICP algorithm uses the minimum distance between the corresponding points of the source point cloud and the target point cloud as the registration criterion. But Point-To-Plane takes minimizing the distance between source point cloud and the plane of the corresponding point in the target point cloud as criterion. Compared with ICP, it can better reflect the spatial structure of the point cloud
, better resist to incorrect point pair. And its iteration speed is faster. Code: [02plane_icp.cpp](02pcl_registration/src/01plane_icp.cpp)
![alt text](picture/point_to_point.jpg)
![alt text](picture/point_to_plane.jpg)