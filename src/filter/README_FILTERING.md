# 点云滤波器使用说明

## 概述
该ROS2节点提供了多种点云滤波方法。PassThrough条件滤波始终启用作为基础，然后可以选择启用一种高级滤波器。选中的高级滤波器结果将用于2D投影和直线检测。

## 滤波器架构
```
原始点云 (/livox/lidar)
    ↓
PassThrough滤波 (条件滤波，始终启用)
    ↓
根据advanced_filter_type参数选择：
├─ none: 直接用PassThrough结果投影到2D
├─ statistical: 统计滤波 → 投影到2D  
├─ bilateral: 双边滤波 → 投影到2D
└─ gaussian: 高斯滤波 → 投影到2D
    ↓
filtered_cloud_2d (用于直线检测)
```

## 可用的滤波器

### 1. PassThrough滤波器（基础滤波）
- **功能**: 根据x、y、z坐标范围过滤点云
- **始终启用**: 这是基础滤波，会首先应用
- **参数**:
  - `x_min`, `x_max`: X轴范围
  - `y_min`, `y_max`: Y轴范围  
  - `z_min`, `z_max`: Z轴范围

### 2. 统计滤波器 (Statistical Outlier Removal)
- **功能**: 移除统计意义上的离群点，减少噪声
- **启用参数**: `advanced_filter_type: 'statistical'`
- **适用场景**: 点云有明显噪声点时
- **参数**:
  - `sor_mean_k`: 分析邻近点的数量（默认50）
  - `sor_stddev_mul`: 标准差倍数阈值（默认1.0，值越小移除越多点）

### 3. 双边滤波器 (Bilateral Filter)
- **功能**: 在平滑点云的同时保持边缘特征
- **启用参数**: `advanced_filter_type: 'bilateral'`
- **适用场景**: 需要平滑点云但保持物体边界清晰时
- **参数**:
  - `bilateral_sigma_s`: 空间标准差，控制平滑程度（默认0.05）
  - `bilateral_sigma_r`: 值域标准差，控制边缘保持（默认0.05）

### 4. 高斯滤波器 (Moving Least Squares)
- **功能**: 使用移动最小二乘法进行表面重建和平滑
- **启用参数**: `advanced_filter_type: 'gaussian'`
- **适用场景**: 需要高质量表面重建时
- **参数**:
  - `gaussian_radius`: 搜索半径（默认0.03）

## 工作模式

### Single Mode (单一模式)
- **用途**: `ros2 run` 或 `ros2 launch` 启动
- **特点**: 只启用选中的一种高级滤波器，节省计算资源
- **配置**: `comparison_mode: false`

### Comparison Mode (比较模式)  
- **用途**: 脚本启动 (`./test_filters.sh`)
- **特点**: 同时启用所有滤波器，便于对比效果
- **配置**: `comparison_mode: true`

### 3. 双边滤波器 (Bilateral Filter)
- **功能**: 在平滑点云的同时保持边缘特征
- **启用参数**: `enable_bilateral_filter: true`
- **适用场景**: 需要平滑点云但保持物体边界清晰时
- **参数**:
  - `bilateral_sigma_s`: 空间标准差，控制平滑程度（默认0.05）
  - `bilateral_sigma_r`: 值域标准差，控制边缘保持（默认0.05）

### 4. 高斯滤波器 (Moving Least Squares)
- **功能**: 使用移动最小二乘法进行表面重建和平滑
- **启用参数**: `enable_gaussian_filter: true`
- **适用场景**: 需要高质量表面重建时
- **参数**:
  - `gaussian_radius`: 搜索半径（默认0.03）

## 使用方法

### 方法1: 修改默认配置文件
编辑 `src/filter/config/filter_params.yaml`：

```yaml
pointcloud_filter_node:
  ros__parameters:
    # 选择高级滤波器类型
    advanced_filter_type: 'statistical'  # none, statistical, bilateral, gaussian
    
    # 调整相应参数
    sor_mean_k: 50
    sor_stddev_mul: 1.0
```

### 方法2: 使用脚本启动比较模式
使用脚本启动所有滤波器进行比较：

```bash
# 比较所有滤波器，使用统计滤波结果进行2D投影和直线检测
./test_filters.sh statistical

# 比较所有滤波器，使用双边滤波结果进行2D投影和直线检测
./test_filters.sh bilateral

# 比较所有滤波器，使用高斯滤波结果进行2D投影和直线检测
./test_filters.sh gaussian

# 比较所有滤波器，使用PassThrough结果进行2D投影和直线检测
./test_filters.sh none
```

### 方法3: 使用ros2 run启动单一模式
直接运行只启用选中的滤波器：

```bash
# 只启用统计滤波器
ros2 run filter pointcloud_filter_node --ros-args \
    --params-file src/filter/config/filter_params.yaml \
    -p advanced_filter_type:=statistical

# 只启用双边滤波器
ros2 run filter pointcloud_filter_node --ros-args \
    --params-file src/filter/config/filter_params.yaml \
    -p advanced_filter_type:=bilateral
```

### 方法4: 运行时动态修改参数
```bash
# 切换到统计滤波器
ros2 param set /pointcloud_filter_node advanced_filter_type statistical

# 切换到双边滤波器  
ros2 param set /pointcloud_filter_node advanced_filter_type bilateral

# 切换到高斯滤波器
ros2 param set /pointcloud_filter_node advanced_filter_type gaussian

# 关闭高级滤波器，只用PassThrough
ros2 param set /pointcloud_filter_node advanced_filter_type none

# 调整滤波器参数
ros2 param set /pointcloud_filter_node sor_mean_k 30
ros2 param set /pointcloud_filter_node bilateral_sigma_s 0.1
```

## 双模式架构说明

本滤波器节点支持两种运行模式：

### 单一模式 (comparison_mode: false)
- 使用场景：ros2 run/launch启动
- 特性：只启用选中的高级滤波器，资源消耗低
- 配置文件：`filter_params.yaml`
- 输出话题：仅发布PassThrough结果和选中的高级滤波器结果

### 比较模式 (comparison_mode: true)  
- 使用场景：脚本启动，用于比较不同滤波器效果
- 特性：同时启用所有高级滤波器，便于效果对比
- 配置文件：`filter_comparison.yaml`
- 输出话题：发布所有滤波器的结果

## 话题说明

### 输入话题
- `/livox/lidar` - 原始点云数据

### 输出话题（根据模式不同而变化）

#### 单一模式输出
- `filtered_cloud_passthrough` - PassThrough条件滤波结果（始终发布）
- `filtered_cloud_[type]` - 选中的高级滤波器结果（仅发布选中类型）
- `filtered_cloud_2d` - 用于直线检测的2D投影点云
- `board_line_marker` - 检测到的直线可视化标记
- `board_bottom_center` - 计算得到的板子底部中心点

#### 比较模式输出
- `filtered_cloud_passthrough` - PassThrough条件滤波结果（始终发布）
- `filtered_cloud_statistical` - 统计滤波结果
- `filtered_cloud_bilateral` - 双边滤波结果
- `filtered_cloud_gaussian` - 高斯滤波结果
- `filtered_cloud_2d` - 用于直线检测的2D投影点云（使用指定滤波器结果）
- `board_line_marker` - 检测到的直线可视化标记
- `board_bottom_center` - 计算得到的板子底部中心点

## 性能对比建议

### 测试不同滤波器效果:
## 配置参数说明

### 模式控制
- `comparison_mode`: 设置运行模式
  - `true`: 比较模式（适用于脚本启动，启用所有滤波器进行比较）
  - `false`: 单一模式（适用于ros2 run/launch，只启用选中的滤波器）

### 滤波器选择
- `advanced_filter_type`: 选择要使用的高级滤波器类型
  - `"statistical"`: 统计滤波器
  - `"bilateral"`: 双边滤波器
  - `"gaussian"`: 高斯滤波器（基于MLS）
  - `"none"`: 只使用PassThrough滤波器

### PassThrough滤波器参数
- `pass_x_min`, `pass_x_max`: X轴过滤范围
- `pass_y_min`, `pass_y_max`: Y轴过滤范围  
- `pass_z_min`, `pass_z_max`: Z轴过滤范围

### 统计滤波器参数
- `sor_mean_k`: 用于计算阈值的邻近点数量
- `sor_stddev_mul`: 标准差乘数阈值

### 双边滤波器参数
- `bilateral_sigma_s`: 空间核的标准差
- `bilateral_sigma_r`: 强度相似性的标准差

### 高斯滤波器参数 
- `mls_search_radius`: 搜索半径
- `mls_polynomial_order`: 多项式拟合阶数

## 性能对比建议

### 单一模式测试:
使用ros2 run可以高效测试单个滤波器：

```bash
# 测试baseline效果
ros2 run filter pointcloud_filter_node --ros-args \
    --params-file src/filter/config/filter_params.yaml \
    -p advanced_filter_type:=none

# 测试统计滤波效果  
ros2 run filter pointcloud_filter_node --ros-args \
    --params-file src/filter/config/filter_params.yaml \
    -p advanced_filter_type:=statistical
```

### 比较模式测试:
使用脚本可以同时比较所有滤波器效果：

```bash
# 比较所有滤波器，使用统计滤波结果进行2D投影
./test_filters.sh statistical

# 比较所有滤波器，使用双边滤波结果进行2D投影
./test_filters.sh bilateral  

# 比较所有滤波器，使用高斯滤波结果进行2D投影
./test_filters.sh gaussian

# 比较所有滤波器，使用PassThrough结果进行2D投影
./test_filters.sh none
```

### 评估指标:
- **噪声减少**: 观察点云中噪声点的减少程度
- **边缘保持**: 检查物体边界是否清晰
- **计算性能**: 监控处理时间和CPU使用率
- **检测精度**: 观察直线检测和中心点计算的稳定性

## 参数调优建议

### 统计滤波器调优:
- 增大 `sor_mean_k` → 更严格的过滤，但计算慢
- 减小 `sor_stddev_mul` → 移除更多点，但可能过滤有用信息

### 双边滤波器调优:
- 增大 `bilateral_sigma_s` → 更强的平滑效果
- 减小 `bilateral_sigma_r` → 更好的边缘保持

### 高斯滤波器调优:
- 增大 `gaussian_radius` → 更平滑的表面，但细节丢失
- 减小 `gaussian_radius` → 保持更多细节，但平滑效果减弱

## 注意事项
1. PassThrough条件滤波始终启用作为基础滤波
2. 高级滤波器一次只能启用一种，通过`advanced_filter_type`参数控制
3. 选中的高级滤波器结果将用于2D投影和直线检测
4. 参数会在运行时每秒刷新一次，支持动态切换滤波器类型
5. 适合比较不同滤波器对直线检测精度的影响
