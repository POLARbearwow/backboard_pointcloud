#!/bin/bash

# 点云滤波器测试脚本 - 比较模式版本
# 特性：启动所有滤波器进行同时比较，可选择用于2D投影的滤波器
# 使用方法: ./test_filters.sh [filter_for_2d]
# filter_for_2d: none, statistical, bilateral, gaussian (选择用于2D投影和直线检测的滤波器)

WORKSPACE_DIR="/home/niu/Desktop/pointcloud_ws"
FILTER_FOR_2D=${1:-"statistical"}

echo "启动点云滤波器 - 比较模式（所有滤波器同时启用）"
echo "用于2D投影和直线检测的滤波器: $FILTER_FOR_2D"

# Source ROS2 workspace
source $WORKSPACE_DIR/install/setup.bash

case $FILTER_FOR_2D in
    "none")
        echo "所有滤波器启用，使用PassThrough结果进行2D投影"
        echo "输出topics:"
        echo "  - filtered_cloud_passthrough (PassThrough结果)"
        echo "  - filtered_cloud_statistical (统计滤波结果)"
        echo "  - filtered_cloud_bilateral (双边滤波结果)"
        echo "  - filtered_cloud_gaussian (高斯滤波结果)"
        echo "  - filtered_cloud_2d (PassThrough结果的2D投影，用于直线检测)"
        ros2 run filter pointcloud_filter_node --ros-args \
            --params-file $WORKSPACE_DIR/src/filter/config/filter_comparison.yaml \
            -p advanced_filter_type:=none
        ;;
    "statistical")
        echo "所有滤波器启用，使用统计滤波结果进行2D投影"
        echo "输出topics:"
        echo "  - filtered_cloud_passthrough (PassThrough结果)"
        echo "  - filtered_cloud_statistical (统计滤波结果)"
        echo "  - filtered_cloud_bilateral (双边滤波结果)"
        echo "  - filtered_cloud_gaussian (高斯滤波结果)"
        echo "  - filtered_cloud_2d (统计滤波结果的2D投影，用于直线检测)"
        ros2 run filter pointcloud_filter_node --ros-args \
            --params-file $WORKSPACE_DIR/src/filter/config/filter_comparison.yaml \
            -p advanced_filter_type:=statistical
        ;;
    "bilateral")
        echo "所有滤波器启用，使用双边滤波结果进行2D投影"
        echo "输出topics:"
        echo "  - filtered_cloud_passthrough (PassThrough结果)"
        echo "  - filtered_cloud_statistical (统计滤波结果)"
        echo "  - filtered_cloud_bilateral (双边滤波结果)"
        echo "  - filtered_cloud_gaussian (高斯滤波结果)"
        echo "  - filtered_cloud_2d (双边滤波结果的2D投影，用于直线检测)"
        ros2 run filter pointcloud_filter_node --ros-args \
            --params-file $WORKSPACE_DIR/src/filter/config/filter_comparison.yaml \
            -p advanced_filter_type:=bilateral
        ;;
    "gaussian")
        echo "所有滤波器启用，使用高斯滤波结果进行2D投影"
        echo "输出topics:"
        echo "  - filtered_cloud_passthrough (PassThrough结果)"
        echo "  - filtered_cloud_statistical (统计滤波结果)"
        echo "  - filtered_cloud_bilateral (双边滤波结果)"
        echo "  - filtered_cloud_gaussian (高斯滤波结果)"
        echo "  - filtered_cloud_2d (高斯滤波结果的2D投影，用于直线检测)"
        ros2 run filter pointcloud_filter_node --ros-args \
            --params-file $WORKSPACE_DIR/src/filter/config/filter_comparison.yaml \
            -p advanced_filter_type:=gaussian
        ;;
    *)
        echo "未知的滤波器类型: $FILTER_FOR_2D"
        echo "可用选项:"
        echo "  none        - 使用PassThrough结果进行2D投影"
        echo "  statistical - 使用统计滤波结果进行2D投影"
        echo "  bilateral   - 使用双边滤波结果进行2D投影"
        echo "  gaussian    - 使用高斯滤波结果进行2D投影"
        echo ""
        echo "注意：无论选择哪个，所有滤波器都会启用并发布到各自的topic"
        exit 1
        ;;
esac
