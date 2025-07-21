// pointcloud_filter.cpp
// ROS2 node that subscribes to /livox/lidar, filters points within configurable x/y/z bounds, and republishes the filtered cloud with the same frame_id.

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/ransac.h>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <Eigen/Dense>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <deque>

class PointCloudFilterNode : public rclcpp::Node
{
public:
  PointCloudFilterNode() : Node("pointcloud_filter_node")
  {
    // Declare and get filter parameters (default bounds keep all points)
    declare_parameter<double>("x_min", -std::numeric_limits<double>::max());
    declare_parameter<double>("x_max",  std::numeric_limits<double>::max());
    declare_parameter<double>("y_min", -std::numeric_limits<double>::max());
    declare_parameter<double>("y_max",  std::numeric_limits<double>::max());
    declare_parameter<double>("z_min", -std::numeric_limits<double>::max());
    declare_parameter<double>("z_max",  std::numeric_limits<double>::max());
    declare_parameter<double>("board_width", 1.8);

    get_filter_params();

    // Create subscription and publisher
    using std::placeholders::_1;
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", rclcpp::SensorDataQoS(), std::bind(&PointCloudFilterNode::cloudCallback, this, _1));

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "filtered_cloud", rclcpp::SensorDataQoS());

    cloud_pub_2d_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "filtered_cloud_2d", rclcpp::SensorDataQoS());

    // Publisher for fitted line visualization in RViz
    line_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "board_line_marker", 10);

    // Publisher for computed center point
    center_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "board_bottom_center", 10);

    // Timer to periodically refresh parameters at runtime
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PointCloudFilterNode::get_filter_params, this));

    history_size_ = 5; // fixed five-frame fusion
  }

private:
  void get_filter_params()
  {
    x_min_ = get_parameter("x_min").as_double();
    x_max_ = get_parameter("x_max").as_double();
    y_min_ = get_parameter("y_min").as_double();
    y_max_ = get_parameter("y_max").as_double();
    z_min_ = get_parameter("z_min").as_double();
    z_max_ = get_parameter("z_max").as_double();
    board_width_ = get_parameter("board_width").as_double();
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convert ROS2 PointCloud2 message to PCL PointCloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    // PassThrough filter on x, y, z
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud);

    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_min_, x_max_);
    pass.filter(*cloud);

    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_min_, y_max_);
    pass.filter(*cloud);

    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min_, z_max_);
    pass.filter(*cloud);

    // Convert back to ROS2 message
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud, output_msg);
    output_msg.header = msg->header; // keep original frame ID and timestamp

    cloud_pub_->publish(output_msg);

    // Create 2D flattened point cloud (z = 0)

    // Flatten to 2D (z = 0)
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2d(new pcl::PointCloud<pcl::PointXYZI>(*cloud));
    for (auto & pt : cloud2d->points)
    {
      pt.z = 0.0f;
    }

    // Remove NaN and near-zero points, keep only ROI points for subsequent processing
    pcl::PointCloud<pcl::PointXYZI>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    roi_cloud->reserve(cloud2d->points.size());
    for (const auto &pt : cloud2d->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;
      if (std::fabs(pt.x) < 1e-4 && std::fabs(pt.y) < 1e-4) continue; // skip (0,0)
      roi_cloud->points.push_back(pt);
    }

    sensor_msgs::msg::PointCloud2 output_msg_2d;
    pcl::toROSMsg(*cloud2d, output_msg_2d);
    output_msg_2d.header = msg->header;

    cloud_pub_2d_->publish(output_msg_2d);

    // Print up to 20 ROI 2D points' x y for debugging
    std::ostringstream ss_pts;
    ss_pts << "ROI 2D points (" << roi_cloud->points.size() << ") sample: ";
    size_t printed = 0;
    for (const auto &pt : roi_cloud->points) {
      ss_pts << "(" << std::fixed << std::setprecision(3) << pt.x << "," << pt.y << ") ";
      if (++printed >= 30) break;
    }
    RCLCPP_INFO(this->get_logger(), "%s", ss_pts.str().c_str());

    // ================= Accumulate frames =================
    cloud_buffer_.push_back(roi_cloud);
    if (cloud_buffer_.size() > history_size_) {
      cloud_buffer_.pop_front();
    }

    // Merge buffered clouds
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto &c : cloud_buffer_) {
      *merged_cloud += *c;
    }

    // ================= Line fitting on merged cloud =================
    if (merged_cloud->points.size() < 30) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Not enough accumulated points for line fitting (%zu)", merged_cloud->points.size());
      return;
    }

    pcl::SampleConsensusModelLine<pcl::PointXYZI>::Ptr model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZI>(merged_cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_l);
    ransac.setDistanceThreshold(0.01); // 1 cm threshold
    ransac.computeModel();

    std::vector<int> inlier_indices;
    ransac.getInliers(inlier_indices);
    if (inlier_indices.size() < 2) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Line RANSAC produced insufficient inliers (%zu)", inlier_indices.size());
      return;
    }

    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff); // [x0 y0 z0 dx dy dz]
    Eigen::Vector3f P0(coeff[0], coeff[1], coeff[2]);
    Eigen::Vector3f dir(coeff[3], coeff[4], coeff[5]);
    dir.normalize();

    // Find endpoints by projecting inliers onto the line
    float t_min = std::numeric_limits<float>::max();
    float t_max = -std::numeric_limits<float>::max();
    for (int idx : inlier_indices) {
      const auto &pt = merged_cloud->points[idx];
      Eigen::Vector3f p(pt.x, pt.y, pt.z);
      float t = dir.dot(p - P0);
      if (t < t_min) t_min = t;
      if (t > t_max) t_max = t;
    }

    // Correct endpoints length to board_width_
    float current_len = t_max - t_min;
    if (std::fabs(current_len - board_width_) > 0.05) {
      float extend = (board_width_ - current_len) / 2.0f;
      // float extend = 0.0f;
      t_min -= extend;
      t_max += extend;
    }

    Eigen::Vector3f P_min = P0 + dir * t_min;
    Eigen::Vector3f P_max = P0 + dir * t_max;
    Eigen::Vector3f center = 0.5f * (P_min + P_max);

    // Publish marker for visualization
    visualization_msgs::msg::Marker line_marker;
    line_marker.header = msg->header;
    line_marker.ns = "board_line";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    geometry_msgs::msg::Point p_msg;
    p_msg.x = P_min.x(); p_msg.y = P_min.y(); p_msg.z = 0.0;
    line_marker.points.push_back(p_msg);
    p_msg.x = P_max.x(); p_msg.y = P_max.y(); p_msg.z = 0.0;
    line_marker.points.push_back(p_msg);
    line_marker.scale.x = 0.02; // line width 2 cm
    line_marker.color.r = 1.0f; line_marker.color.g = 0.0f; line_marker.color.b = 0.0f; line_marker.color.a = 1.0f;
    line_marker.lifetime = rclcpp::Duration::from_seconds(0); // persistent
    line_marker_pub_->publish(line_marker);

    // Publish center point
    geometry_msgs::msg::PointStamped center_msg;
    center_msg.header = msg->header;
    center_msg.point.x = center.x();
    center_msg.point.y = center.y();
    center_msg.point.z = 0.0;
    center_pub_->publish(center_msg);

    RCLCPP_INFO(this->get_logger(), "Board bottom center: (%.3f, %.3f)", center.x(), center.y());
  }

  // Members
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_2d_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr center_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // buffer of recent ROI clouds
  std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_buffer_;
  size_t history_size_;
  // Filter bounds
  double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
  double board_width_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudFilterNode>());
  rclcpp::shutdown();
  return 0;
} 