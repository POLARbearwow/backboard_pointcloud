# 2D 点云中自动获取篮板底边中心 (x, y) —— 方案规划

> 适用输入：已按 z=0 投影、仅包含“篮板底边 + 篮筐圆环”及少量噪声的 `filtered_cloud_2d` (sensor_msgs/PointCloud2)

---

## 目标
1. 在 XY 平面中找到 **篮板底边直线**。
2. 根据已知篮板宽度 (1.80 m)，计算底边 **两端点**。
3. 取两端点的中点 → 输出篮板底边中心 `(x_c, y_c)`。

---

## 总体流程
```text
1. 预处理
   ├─ （可选）StatisticalOutlierRemoval 去孤立点（如有噪声）
2. 直线检测 (RANSAC)
   ├─ 模型: y = kx + b or ax + by + c = 0
   ├─ distThreshold: 0.01 m
   ├─ 得到内点集合 bottom_line_cloud
3. 精炼端点
   ├─ PCA / SVD → 主方向向量 v
   ├─ 在 v 上投影 bottom_line_cloud
   ├─ 取投影最小/最大点 P_min, P_max
4. 端点矫正
   ├─ 已知理论宽度 W = 1.80 m
   ├─ 实测 d = |P_max - P_min|
   ├─ 若 |d - W| < 0.05 m, 直接用
   └─ 否则向两端沿 v 单侧延伸 (W - d)/2 补足
5. 计算中心
   center = (P_min + P_max) / 2
6. 发布结果
   ├─ TF: child_frame_id="board_bottom_center"
   └─ 或 topic geometry_msgs/Point
```

---

## 关键实现细节

1. **RANSAC 直线 vs. 拟合所有点取平均**  
   - 直接平均可能受篮筐/噪声干扰；  
   - RANSAC 专门挑选一条“密集 & 最长”的直线 → 鲁棒。
2. **PCA 端点获取**  
   对直线内点做 PCA，第一主成分即直线方向向量 `v`，再用一维投影快速找端点；比在 XY 平面做断层检测更简单。
3. **宽度矫正**  
   雷达可只扫到篮板部分长度，利用已知 **1.80 m** 可推断真实端点，提升准确度。
4. **多帧滤波**  
   将 `(x_c, y_c)` 送入卡尔曼/滑窗均值，平滑抖动。
5. **实时性**  
   全流程仅在 2D 点集上做 RANSAC + PCA，计算量很小，可达 kHz。

---

## 备选方案
| 方法 | 思路 | 适用场景 | 优缺点 |
|------|------|-----------|---------|
| Hough Transform | 在网格图像中检测最长直线 | 点数稀疏、噪声多 | 容忍噪声高，但需栅格化，分辨率影响精度 |
| 投影-直方图 | 将点云在 Y (或 X) 方向直方图，找峰 | 相机视角不变，底边几乎水平 | 算法更简单，但不通用于倾斜视角 |

---

## 输出接口示例 (C++)
```cpp
geometry_msgs::msg::Point pt;
pt.x = center.x;
pt.y = center.y;
pt.z = 0.0;             // 已在 XY 平面
center_pub_->publish(pt);
```

如需后续在 3D 坐标系可用 TF：
```cpp
geometry_msgs::msg::TransformStamped t;
... // header
 t.transform.translation.x = pt.x;
 t.transform.translation.y = pt.y;
 t.transform.translation.z = pt.z;
 t.transform.rotation = identity_quat;
```

---

### 下一步
1. 在 `filter` 包内新增 `board_center_node.cpp` 实现以上流程；  
2. 在 YAML 中加入参数：RANSAC 阈值、宽度、下采样分辨率等；  
3. 在同一 launch 文件中同时启动滤波节点与中心定位节点。 