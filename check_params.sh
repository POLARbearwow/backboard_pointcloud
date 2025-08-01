#!/bin/bash

echo "=== ROS2 参数检查脚本 ==="
echo

# 检查节点是否在运行
echo "1. 检查节点状态:"
ros2 node list | grep pointcloud_filter_node || echo "❌ 节点未运行"

echo
echo "2. 查看所有参数:"
ros2 param list /pointcloud_filter_node

echo
echo "3. 检查PassThrough滤波器参数:"
echo "x_min: $(ros2 param get /pointcloud_filter_node x_min)"
echo "x_max: $(ros2 param get /pointcloud_filter_node x_max)"
echo "y_min: $(ros2 param get /pointcloud_filter_node y_min)"
echo "y_max: $(ros2 param get /pointcloud_filter_node y_max)"
echo "z_min: $(ros2 param get /pointcloud_filter_node z_min)"
echo "z_max: $(ros2 param get /pointcloud_filter_node z_max)"

echo
echo "4. 检查滤波器类型:"
echo "filter_type: $(ros2 param get /pointcloud_filter_node filter_type)"
echo "advanced_filter_type: $(ros2 param get /pointcloud_filter_node advanced_filter_type)"
echo "comparison_mode: $(ros2 param get /pointcloud_filter_node comparison_mode)"

echo
echo "5. 检查聚类参数:"
echo "cluster_tolerance: $(ros2 param get /pointcloud_filter_node cluster_tolerance)"
echo "min_cluster_size: $(ros2 param get /pointcloud_filter_node min_cluster_size)"
echo "max_cluster_size: $(ros2 param get /pointcloud_filter_node max_cluster_size)"

echo
echo "6. 导出当前所有参数到文件:"
ros2 param dump /pointcloud_filter_node > /tmp/current_filter_params.yaml
echo "参数已保存到: /tmp/current_filter_params.yaml"

echo
echo "=== 检查完成 ==="
