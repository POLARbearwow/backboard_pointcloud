# RViz配置指南 - 多滤波器同时比较

## 快速设置步骤

### 1. 启动RViz
```bash
rviz2
```

### 2. 添加PointCloud2显示
依次添加4个PointCloud2显示，配置如下：

#### 显示1: PassThrough基准
- **名称**: PassThrough
- **Topic**: `/pointcloud_filter_node/filtered_cloud_passthrough`
- **颜色**: 红色 (RGB: 255, 0, 0)
- **Point Size**: 2

#### 显示2: 统计滤波
- **名称**: Statistical  
- **Topic**: `/pointcloud_filter_node/filtered_cloud_statistical`
- **颜色**: 绿色 (RGB: 0, 255, 0)
- **Point Size**: 2

#### 显示3: 双边滤波
- **名称**: Bilateral
- **Topic**: `/pointcloud_filter_node/filtered_cloud_bilateral`
- **颜色**: 蓝色 (RGB: 0, 0, 255)
- **Point Size**: 2

#### 显示4: 高斯滤波
- **名称**: Gaussian
- **Topic**: `/pointcloud_filter_node/filtered_cloud_gaussian`
- **颜色**: 黄色 (RGB: 255, 255, 0)
- **Point Size**: 2

### 3. 添加其他显示
- **Marker**: `/pointcloud_filter_node/board_line_marker` (直线检测结果)
- **PointStamped**: `/pointcloud_filter_node/board_bottom_center` (中心点)

### 4. 设置Fixed Frame
- **Fixed Frame**: `livox_frame` 或根据您的激光雷达frame设置

## 比较分析要点

### 观察指标
1. **点云密度**: 哪种滤波器保留了最多有用信息
2. **噪声去除**: 哪种滤波器最有效去除了噪声点
3. **边缘保持**: 物体边界是否清晰
4. **表面平滑度**: 表面是否过度平滑导致细节丢失

### 动态调整参数
运行时可以动态调整参数观察效果变化：

```bash
# 调整统计滤波器
ros2 param set /pointcloud_filter_node sor_mean_k 30
ros2 param set /pointcloud_filter_node sor_stddev_mul 0.8

# 调整双边滤波器
ros2 param set /pointcloud_filter_node bilateral_sigma_s 0.1
ros2 param set /pointcloud_filter_node bilateral_sigma_r 0.1

# 调整高斯滤波器
ros2 param set /pointcloud_filter_node gaussian_radius 0.05

# 启用/禁用滤波器
ros2 param set /pointcloud_filter_node enable_statistical_filter false
ros2 param set /pointcloud_filter_node enable_bilateral_filter true
```

## 保存RViz配置
配置完成后，建议保存RViz配置文件：
**File → Save Config As...** → `filter_comparison.rviz`

下次直接加载：
```bash
rviz2 -d filter_comparison.rviz
```

## 性能监控
观察终端输出，关注：
- 各滤波器处理的点数
- 处理时间差异
- 内存使用情况

## 最佳实践
1. 先用单个滤波器测试，确认效果
2. 逐步启用其他滤波器进行对比
3. 根据具体应用场景选择最优组合
4. 记录最佳参数配置便于后续使用
