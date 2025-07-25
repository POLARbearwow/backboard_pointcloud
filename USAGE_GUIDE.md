# 点云滤波器比较系统 - 使用指南

## 🎯 系统特性
- **单一高级滤波器选择**: PassThrough始终启用，高级滤波器一次只启用一种
- **基于条件滤波**: 高级滤波器基于PassThrough滤波结果进行处理
- **2D投影优化**: 使用选中的高级滤波器结果进行2D投影和直线检测
- **参数化配置**: 通过`advanced_filter_type`参数选择滤波器类型
- **运行时切换**: 支持动态切换不同滤波器类型

## 🚀 快速开始

### 1. 编译项目
```bash
cd /home/niu/Desktop/pointcloud_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. 启动滤波器（推荐）
```bash
# 使用便捷脚本测试统计滤波器
./test_filters.sh statistical

# 或直接运行默认配置
ros2 run filter pointcloud_filter_node --ros-args \
    --params-file src/filter/config/filter_params.yaml
```

### 3. 启动RViz进行可视化
```bash
rviz2
# 重点观察：
# - filtered_cloud_passthrough (PassThrough基准)
# - filtered_cloud_[selected] (选中的高级滤波器结果)  
# - filtered_cloud_2d (用于直线检测的2D投影)
# - board_line_marker (直线检测结果)
```

## 📊 输出Topic说明

| Topic名称 | 内容 | 用途 |
|-----------|------|------|
| `filtered_cloud_passthrough` | PassThrough条件滤波结果 | 始终发布的基准结果 |
| `filtered_cloud_statistical` | 统计滤波结果 | 仅在`advanced_filter_type='statistical'`时发布 |
| `filtered_cloud_bilateral` | 双边滤波结果 | 仅在`advanced_filter_type='bilateral'`时发布 |
| `filtered_cloud_gaussian` | 高斯滤波结果 | 仅在`advanced_filter_type='gaussian'`时发布 |
| `filtered_cloud_2d` | 2D投影点云 | 使用选中的高级滤波器结果投影 |
| `board_line_marker` | 直线可视化标记 | 基于2D投影的检测结果 |
| `board_bottom_center` | 板子中心点 | 基于2D投影的检测结果 |

## ⚙️ 配置文件说明

### 主配置文件: `src/filter/config/filter_params.yaml`
- 包含`advanced_filter_type`参数选择滤波器
- 默认使用`statistical`滤波器

### 示例配置文件: `src/filter/config/filter_examples.yaml`
- 包含4种不同滤波器的配置示例
- 适合快速切换测试

### 测试脚本: `test_filters.sh`
- 便捷的启动脚本
- 支持不同配置模式

## 🔧 参数配置

### 滤波器选择参数
```yaml
advanced_filter_type: 'statistical'  # none, statistical, bilateral, gaussian
```

### 统计滤波器 (去噪声)
```yaml
sor_mean_k: 50          # 邻近点数量 (30-100)
sor_stddev_mul: 1.0     # 标准差倍数 (0.5-2.0)
```

### 双边滤波器 (边缘保持平滑)
```yaml
bilateral_sigma_s: 0.05    # 空间标准差 (0.01-0.2)
bilateral_sigma_r: 0.05    # 值域标准差 (0.01-0.2)
```

### 高斯滤波器 (表面重建)
```yaml
gaussian_radius: 0.03      # 搜索半径 (0.01-0.1)
```

## 📈 性能比较建议

### 评估维度
1. **直线检测稳定性** - 观察`board_line_marker`的一致性
2. **中心点精度** - 监测`board_bottom_center`的准确性
3. **噪声去除效果** - 比较不同滤波器的去噪能力
4. **计算效率** - PassThrough > 统计 > 双边 > 高斯
5. **2D投影质量** - 观察`filtered_cloud_2d`的清晰度

### 测试流程
```bash
# 1. 测试基准（无高级滤波）
./test_filters.sh none
# 记录直线检测稳定性

# 2. 测试统计滤波器
./test_filters.sh statistical  
# 比较噪声去除效果

# 3. 测试双边滤波器
./test_filters.sh bilateral
# 观察边缘保持效果

# 4. 测试高斯滤波器
./test_filters.sh gaussian
# 评估表面重建效果
```

## 🛠️ 故障排除

### 编译错误
```bash
# 确保安装了PCL依赖
sudo apt install libpcl-dev

# 清理重新编译
rm -rf build install log
colcon build --symlink-install
```

### 运行时问题
```bash
# 检查topic发布状态
ros2 topic list | grep filtered_cloud

# 检查参数设置
ros2 param list /pointcloud_filter_node

# 监控处理性能
ros2 topic hz /pointcloud_filter_node/filtered_cloud_passthrough
```

### RViz显示问题
- 确认Fixed Frame设置正确
- 检查topic名称是否匹配
- 验证点云数据是否发布

## 📝 实验记录建议

记录不同配置下的效果：

| 配置 | 噪声去除 | 边缘保持 | 表面平滑 | 计算时间 | 直线检测精度 |
|------|----------|----------|----------|----------|--------------|
| PassThrough | ⭐⭐ | ⭐⭐⭐ | ⭐⭐ | ⭐⭐⭐⭐⭐ | 基准 |
| +Statistical | ⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐ | ⭐⭐⭐⭐ | ? |
| +Bilateral | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐ | ? |
| +Gaussian | ⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐ | ? |

## 📚 参考文档
- [详细滤波器说明](src/filter/README_FILTERING.md)
- [RViz配置指南](RViz_Setup_Guide.md)
- [配置文件示例](src/filter/config/filter_examples.yaml)
