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
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/bilateral.h>
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>


#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <Eigen/Dense>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <deque>
#include <algorithm>

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
    declare_parameter<double>("len_tol", 0.3);        // acceptable deviation in meters
    declare_parameter<double>("min_inlier_ratio", 0.3); // min inliers percentage
    declare_parameter<int>("max_candidates", 3); // number of line candidates to evaluate
    declare_parameter<int>("history_size", 5);
    declare_parameter<double>("ransac_dist_thresh", 0.02);
    declare_parameter<std::string>("filter_type", "passthrough");
    declare_parameter<std::string>("advanced_filter_type", "none");  // none, statistical, bilateral, gaussian
    declare_parameter<bool>("comparison_mode", false);  // true: 启动所有滤波器比较, false: 只启动选中的滤波器
    declare_parameter<int>("sor_mean_k", 50);
    declare_parameter<double>("sor_stddev_mul", 1.0);
    declare_parameter<double>("bilateral_sigma_s", 0.05);
    declare_parameter<double>("bilateral_sigma_r", 0.05);
    declare_parameter<double>("gaussian_radius", 0.03);
    declare_parameter<double>("cluster_tolerance", 0.15);
    declare_parameter<int>("min_cluster_size", 30);
    declare_parameter<int>("max_cluster_size", 800);
    declare_parameter<int>("max_clusters", 10);
    declare_parameter<double>("max_cluster_dimension", 3.0);
    declare_parameter<double>("min_cluster_dimension", 0.5);

    get_filter_params();

    // Create subscription and publisher
    using std::placeholders::_1;
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", rclcpp::SensorDataQoS(), std::bind(&PointCloudFilterNode::cloudCallback, this, _1));

    // Create publishers for different filtering results
    cloud_pub_passthrough_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "filtered_cloud_passthrough", rclcpp::SensorDataQoS());
    
    cloud_pub_statistical_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "filtered_cloud_statistical", rclcpp::SensorDataQoS());
      
    cloud_pub_bilateral_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "filtered_cloud_bilateral", rclcpp::SensorDataQoS());
      
    cloud_pub_gaussian_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "filtered_cloud_gaussian", rclcpp::SensorDataQoS());

    cloud_pub_2d_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "filtered_cloud_2d", rclcpp::SensorDataQoS());

    // Publisher for clustering results
    cloud_pub_clusters_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "clustered_clouds", rclcpp::SensorDataQoS());
      
    cloud_pub_selected_cluster_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "selected_backboard_cluster", rclcpp::SensorDataQoS());

    // Publisher for fitted line visualization in RViz
    line_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "board_line_marker", 10);

    // Publisher for computed center point
    center_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "board_bottom_center", 10);

    // Publisher for cluster markers
    cluster_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "cluster_markers", 10);

    // Timer to periodically refresh parameters at runtime
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PointCloudFilterNode::get_filter_params, this));

    history_size_ = static_cast<size_t>(get_parameter("history_size").as_int());
    ransac_dist_thresh_ = get_parameter("ransac_dist_thresh").as_double();
    max_candidates_ = get_parameter("max_candidates").as_int();

    filter_type_ = get_parameter("filter_type").as_string();
    advanced_filter_type_ = get_parameter("advanced_filter_type").as_string();
    comparison_mode_ = get_parameter("comparison_mode").as_bool();
    sor_mean_k_ = get_parameter("sor_mean_k").as_int();
    sor_stddev_mul_ = get_parameter("sor_stddev_mul").as_double();
    bilateral_sigma_s_ = get_parameter("bilateral_sigma_s").as_double();
    bilateral_sigma_r_ = get_parameter("bilateral_sigma_r").as_double();
    gaussian_radius_ = get_parameter("gaussian_radius").as_double();
    cluster_tolerance_ = get_parameter("cluster_tolerance").as_double();
    min_cluster_size_ = get_parameter("min_cluster_size").as_int();
    max_cluster_size_ = get_parameter("max_cluster_size").as_int();
    max_clusters_ = get_parameter("max_clusters").as_int();
    max_cluster_dimension_ = get_parameter("max_cluster_dimension").as_double();
    min_cluster_dimension_ = get_parameter("min_cluster_dimension").as_double();
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
    len_tol_ = get_parameter("len_tol").as_double();
    min_inlier_ratio_ = get_parameter("min_inlier_ratio").as_double();
    max_candidates_ = get_parameter("max_candidates").as_int();
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr applyAllFiltersForComparison(pcl::PointCloud<pcl::PointXYZI>::Ptr base_cloud, const std_msgs::msg::Header& header)
  {
    if (base_cloud->points.empty()) {
      RCLCPP_WARN(this->get_logger(), "Base cloud is empty, returning PassThrough result");
      return base_cloud;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr selected_result(new pcl::PointCloud<pcl::PointXYZI>(*base_cloud));

    // Apply and publish all filters
    // Statistical Outlier Removal Filter
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr statistical_cloud(new pcl::PointCloud<pcl::PointXYZI>(*base_cloud));
      pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
      sor.setInputCloud(statistical_cloud);
      sor.setMeanK(sor_mean_k_);
      sor.setStddevMulThresh(sor_stddev_mul_);
      sor.filter(*statistical_cloud);
      
      sensor_msgs::msg::PointCloud2 statistical_msg;
      pcl::toROSMsg(*statistical_cloud, statistical_msg);
      statistical_msg.header = header;
      cloud_pub_statistical_->publish(statistical_msg);
      
      if (advanced_filter_type_ == "statistical") {
        selected_result = statistical_cloud;
      }
      
      RCLCPP_DEBUG(this->get_logger(), "Published statistical filtered cloud with %zu points", statistical_cloud->points.size());
    }

    // Bilateral Filter
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr bilateral_cloud(new pcl::PointCloud<pcl::PointXYZI>(*base_cloud));
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::BilateralFilter<pcl::PointXYZI> bilateral;
      bilateral.setInputCloud(bilateral_cloud);
      bilateral.setHalfSize(bilateral_sigma_s_);
      bilateral.setStdDev(bilateral_sigma_r_);
      bilateral.filter(*filtered_cloud);
      
      sensor_msgs::msg::PointCloud2 bilateral_msg;
      pcl::toROSMsg(*filtered_cloud, bilateral_msg);
      bilateral_msg.header = header;
      cloud_pub_bilateral_->publish(bilateral_msg);
      
      if (advanced_filter_type_ == "bilateral") {
        selected_result = filtered_cloud;
      }
      
      RCLCPP_DEBUG(this->get_logger(), "Published bilateral filtered cloud with %zu points", filtered_cloud->points.size());
    }

    // Gaussian Filter (using Moving Least Squares)
    if (gaussian_radius_ > 0.0) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr gaussian_cloud(new pcl::PointCloud<pcl::PointXYZI>(*base_cloud));
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointXYZI> mls;
      pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
      
      mls.setInputCloud(gaussian_cloud);
      mls.setSearchMethod(tree);
      mls.setSearchRadius(gaussian_radius_);
      mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointXYZI>::NONE);
      mls.process(*filtered_cloud);
      
      sensor_msgs::msg::PointCloud2 gaussian_msg;
      pcl::toROSMsg(*filtered_cloud, gaussian_msg);
      gaussian_msg.header = header;
      cloud_pub_gaussian_->publish(gaussian_msg);
      
      if (advanced_filter_type_ == "gaussian") {
        selected_result = filtered_cloud;
      }
      
      RCLCPP_DEBUG(this->get_logger(), "Published Gaussian (MLS) filtered cloud with %zu points", filtered_cloud->points.size());
    }

    RCLCPP_DEBUG(this->get_logger(), "Comparison mode: published all filters, using %s for 2D processing", advanced_filter_type_.c_str());
    return selected_result;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr applySelectedAdvancedFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr base_cloud, const std_msgs::msg::Header& header)
  {
    if (base_cloud->points.empty()) {
      RCLCPP_WARN(this->get_logger(), "Base cloud is empty, returning PassThrough result");
      return base_cloud;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZI>(*base_cloud));

    if (advanced_filter_type_ == "statistical") {
      // Statistical Outlier Removal Filter
      pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
      sor.setInputCloud(result_cloud);
      sor.setMeanK(sor_mean_k_);
      sor.setStddevMulThresh(sor_stddev_mul_);
      sor.filter(*result_cloud);
      
      sensor_msgs::msg::PointCloud2 statistical_msg;
      pcl::toROSMsg(*result_cloud, statistical_msg);
      statistical_msg.header = header;
      cloud_pub_statistical_->publish(statistical_msg);
      
      RCLCPP_DEBUG(this->get_logger(), "Applied and published statistical filter with %zu points", result_cloud->points.size());
      
    } else if (advanced_filter_type_ == "bilateral") {
      // Bilateral Filter
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::BilateralFilter<pcl::PointXYZI> bilateral;
      bilateral.setInputCloud(result_cloud);
      bilateral.setHalfSize(bilateral_sigma_s_);
      bilateral.setStdDev(bilateral_sigma_r_);
      bilateral.filter(*filtered_cloud);
      result_cloud = filtered_cloud;
      
      sensor_msgs::msg::PointCloud2 bilateral_msg;
      pcl::toROSMsg(*result_cloud, bilateral_msg);
      bilateral_msg.header = header;
      cloud_pub_bilateral_->publish(bilateral_msg);
      
      RCLCPP_DEBUG(this->get_logger(), "Applied and published bilateral filter with %zu points", result_cloud->points.size());
      
    } else if (advanced_filter_type_ == "gaussian") {
      // Gaussian Filter (using Moving Least Squares)
      if (gaussian_radius_ > 0.0) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointXYZI> mls;
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        
        mls.setInputCloud(result_cloud);
        mls.setSearchMethod(tree);
        mls.setSearchRadius(gaussian_radius_);
        mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointXYZI>::NONE);
        mls.process(*filtered_cloud);
        result_cloud = filtered_cloud;
        
        sensor_msgs::msg::PointCloud2 gaussian_msg;
        pcl::toROSMsg(*result_cloud, gaussian_msg);
        gaussian_msg.header = header;
        cloud_pub_gaussian_->publish(gaussian_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Applied and published Gaussian (MLS) filter with %zu points", result_cloud->points.size());
      } else {
        RCLCPP_WARN(this->get_logger(), "Gaussian filter radius is 0, using PassThrough result");
      }
    } else if (advanced_filter_type_ == "none") {
      RCLCPP_DEBUG(this->get_logger(), "No advanced filter selected, using PassThrough result for 2D processing");
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown advanced filter type: %s, using PassThrough result", advanced_filter_type_.c_str());
    }

    return result_cloud;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr performClustering(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud, const std_msgs::msg::Header& header)
  {
    if (input_cloud->points.size() < min_cluster_size_) {
      RCLCPP_WARN(this->get_logger(), "Input cloud too small for clustering (%zu points)", input_cloud->points.size());
      return nullptr;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(input_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    ec.extract(cluster_indices);

    RCLCPP_INFO(this->get_logger(), "Found %zu clusters", cluster_indices.size());

    if (cluster_indices.empty()) {
      return nullptr;
    }

    // Create a combined cloud of all valid clusters for visualization
    pcl::PointCloud<pcl::PointXYZI>::Ptr all_clusters_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    
    // Find the best cluster (smallest one with valid dimensions, likely to be the backboard)
    pcl::PointCloud<pcl::PointXYZI>::Ptr best_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    size_t best_cluster_size = max_cluster_size_ + 1; // Initialize with a size larger than max
    int best_cluster_idx = -1;
    int valid_clusters_count = 0;

    for (size_t i = 0; i < std::min(cluster_indices.size(), static_cast<size_t>(max_clusters_)); ++i) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      
      for (const auto& idx : cluster_indices[i].indices) {
        cluster_cloud->points.push_back(input_cloud->points[idx]);
      }
      cluster_cloud->width = cluster_cloud->points.size();
      cluster_cloud->height = 1;
      cluster_cloud->is_dense = true;

      // Calculate cluster dimensions (bounding box)
      if (cluster_cloud->points.empty()) continue;
      
      float min_x = cluster_cloud->points[0].x, max_x = cluster_cloud->points[0].x;
      float min_y = cluster_cloud->points[0].y, max_y = cluster_cloud->points[0].y;
      
      for (const auto& point : cluster_cloud->points) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
      }
      
      float cluster_width = max_x - min_x;
      float cluster_height = max_y - min_y;
      float cluster_max_dimension = std::max(cluster_width, cluster_height);
      float cluster_min_dimension = std::min(cluster_width, cluster_height);
      
      RCLCPP_INFO(this->get_logger(), "Cluster %zu: %zu points, dimensions: %.2f x %.2f m (max: %.2f m)", 
                  i, cluster_cloud->points.size(), cluster_width, cluster_height, cluster_max_dimension);

      // Check if cluster dimensions are within valid range
      if (cluster_max_dimension > max_cluster_dimension_ || cluster_max_dimension < min_cluster_dimension_) {
        RCLCPP_INFO(this->get_logger(), "Cluster %zu rejected: dimension %.2f m outside range [%.2f, %.2f] m", 
                    i, cluster_max_dimension, min_cluster_dimension_, max_cluster_dimension_);
        continue;
      }
      
      valid_clusters_count++;
      RCLCPP_INFO(this->get_logger(), "Cluster %zu accepted: valid dimensions", i);

      // Add to combined visualization cloud
      *all_clusters_cloud += *cluster_cloud;

      // Check if this is the best cluster (smallest valid one, likely to be backboard)
      if (cluster_cloud->points.size() < best_cluster_size) {
        best_cluster_size = cluster_cloud->points.size();
        best_cluster = cluster_cloud;
        best_cluster_idx = i;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Found %d valid clusters out of %zu total clusters", 
                valid_clusters_count, cluster_indices.size());

    // Publish all clusters for visualization
    if (!cluster_indices.empty()) {
      visualization_msgs::msg::MarkerArray marker_array;
      int id_counter = 0;
      for (size_t i = 0; i < std::min(cluster_indices.size(), static_cast<size_t>(max_clusters_)); ++i) {
        // bounding box already computed earlier, recompute quickly
        if (cluster_indices[i].indices.empty()) continue;
        float min_x = std::numeric_limits<float>::max(), max_x = -std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max(), max_y = -std::numeric_limits<float>::max();
        for (int idx : cluster_indices[i].indices) {
          const auto &pt = input_cloud->points[idx];
          min_x = std::min(min_x, pt.x);
          max_x = std::max(max_x, pt.x);
          min_y = std::min(min_y, pt.y);
          max_y = std::max(max_y, pt.y);
        }
        float cx = 0.5f * (min_x + max_x);
        float cy = 0.5f * (min_y + max_y);
        float width = max_x - min_x;
        float height = max_y - min_y;

        visualization_msgs::msg::Marker box;
        box.header = header;
        box.ns = "clusters";
        box.id = id_counter++;
        box.type = visualization_msgs::msg::Marker::CUBE;
        box.action = visualization_msgs::msg::Marker::ADD;
        box.pose.position.x = cx;
        box.pose.position.y = cy;
        box.pose.position.z = 0.0;
        box.pose.orientation.w = 1.0;
        box.scale.x = width;
        box.scale.y = height;
        box.scale.z = 0.05; // thin box
        // color coding: selected cluster green, others blue
        if (static_cast<int>(i) == best_cluster_idx) {
          box.color.r = 0.0f; box.color.g = 1.0f; box.color.b = 0.0f; box.color.a = 0.6f;
        } else {
          box.color.r = 0.0f; box.color.g = 0.0f; box.color.b = 1.0f; box.color.a = 0.3f;
        }
        marker_array.markers.push_back(box);
      }
      cluster_marker_pub_->publish(marker_array);
    }

    // Publish the selected best cluster
    if (best_cluster && best_cluster->points.size() > 0 && best_cluster_idx >= 0) {
      sensor_msgs::msg::PointCloud2 selected_msg;
      pcl::toROSMsg(*best_cluster, selected_msg);
      selected_msg.header = header;
      cloud_pub_selected_cluster_->publish(selected_msg);
      
      RCLCPP_INFO(this->get_logger(), "Selected cluster %d with %zu points for backboard detection (passed dimension check)", 
                  best_cluster_idx, best_cluster->points.size());
      return best_cluster;
    }

    if (valid_clusters_count == 0) {
      RCLCPP_WARN(this->get_logger(), "No clusters passed dimension validation");
    }

    return nullptr;
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

    // Publish PassThrough filtered result
    sensor_msgs::msg::PointCloud2 passthrough_msg;
    pcl::toROSMsg(*cloud, passthrough_msg);
    passthrough_msg.header = msg->header;
    cloud_pub_passthrough_->publish(passthrough_msg);

    // Apply filters based on mode
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_for_2d;
    if (comparison_mode_) {
      // Comparison mode: publish all filters and use selected one for 2D
      cloud_for_2d = applyAllFiltersForComparison(cloud, msg->header);
    } else {
      // Single mode: only apply selected filter
      cloud_for_2d = applySelectedAdvancedFilter(cloud, msg->header);
    }

    // Convert back to ROS2 message (for 2D processing, use selected result)
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_for_2d, output_msg);
    output_msg.header = msg->header; // keep original frame ID and timestamp

    // Create 2D flattened point cloud (z = 0) using selected filtered result

    // Flatten to 2D (z = 0)
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2d(new pcl::PointCloud<pcl::PointXYZI>(*cloud_for_2d));
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

    // ================= Clustering for backboard detection =================
    pcl::PointCloud<pcl::PointXYZI>::Ptr selected_cluster_cloud = performClustering(roi_cloud, msg->header);
    
    // Use the selected cluster for line detection instead of all ROI points
    if (selected_cluster_cloud && selected_cluster_cloud->points.size() > 0) {
      roi_cloud = selected_cluster_cloud;  // Replace roi_cloud with selected cluster
      RCLCPP_INFO(this->get_logger(), "Using selected cluster with %zu points for line detection", roi_cloud->points.size());
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "No suitable cluster found, using all ROI points (%zu)", roi_cloud->points.size());
    }

    // Print up to 20 ROI 2D points' x y for debugging
    std::ostringstream ss_pts;
    // ss_pts << "ROI 2D points (" << roi_cloud->points.size() << ") sample: ";
    // size_t printed = 0;
    // for (const auto &pt : roi_cloud->points) {
    //   ss_pts << "(" << std::fixed << std::setprecision(3) << pt.x << "," << pt.y << ") ";
    //   if (++printed >= 30) break;
    // }
    // RCLCPP_INFO(this->get_logger(), "%s", ss_pts.str().c_str());

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

    // -------- Iterative RANSAC to find multiple candidate lines --------
    struct Candidate {
      Eigen::Vector3f P0;
      Eigen::Vector3f dir;
      float t_min;
      float t_max;
      size_t inlier_cnt;
    } best_candidate;

    bool found = false;
    int candidate_idx = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZI>(*merged_cloud));

    for (int c = 0; c < max_candidates_; ++c) {
      if (remaining->points.size() < 30) break;

      candidate_idx = c;

      pcl::SampleConsensusModelLine<pcl::PointXYZI>::Ptr model(new pcl::SampleConsensusModelLine<pcl::PointXYZI>(remaining));
      pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model);
      ransac.setDistanceThreshold(ransac_dist_thresh_);
      ransac.computeModel();

      std::vector<int> inliers;
      ransac.getInliers(inliers);
      if (inliers.size() < 10) break; // too few

      Eigen::VectorXf coeff_c;
      ransac.getModelCoefficients(coeff_c);
      Eigen::Vector3f P0_c(coeff_c[0], coeff_c[1], coeff_c[2]);
      Eigen::Vector3f dir_c(coeff_c[3], coeff_c[4], coeff_c[5]);
      dir_c.normalize();

      float tmin_c = std::numeric_limits<float>::max();
      float tmax_c = -std::numeric_limits<float>::max();
      for (int idx : inliers) {
        const auto &pt = remaining->points[idx];
        Eigen::Vector3f p(pt.x, pt.y, pt.z);
        float t = dir_c.dot(p - P0_c);
        tmin_c = std::min(tmin_c, t);
        tmax_c = std::max(tmax_c, t);
      }

      float len_c = tmax_c - tmin_c;
      double inlier_ratio_c = static_cast<double>(inliers.size()) / remaining->points.size();

      // Log candidate statistics
      RCLCPP_INFO(this->get_logger(), "Candidate %d: len=%.3f m, inlier_ratio=%.2f, inliers=%zu/%zu", c, len_c, inlier_ratio_c, inliers.size(), remaining->points.size());

      // evaluate candidate
      if (std::fabs(len_c - board_width_) <= len_tol_ && inlier_ratio_c >= min_inlier_ratio_) {
        best_candidate = {P0_c, dir_c, tmin_c, tmax_c, inliers.size()};
        found = true;
        RCLCPP_INFO(this->get_logger(), "--> Candidate %d fully validated (len=%.3f m, inlier_ratio=%.3f)", 
                    c, len_c, inlier_ratio_c);
        break; // accept first good candidate
      }

      // remove inliers and continue searching
      pcl::ExtractIndices<pcl::PointXYZI> extract;
      pcl::PointIndices::Ptr pi(new pcl::PointIndices);
      pi->indices = inliers;
      extract.setInputCloud(remaining);
      extract.setIndices(pi);
      extract.setNegative(true);
      pcl::PointCloud<pcl::PointXYZI> tmp;
      extract.filter(tmp);
      *remaining = tmp;
    }

    if (!found) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "No line candidate passed validation after testing %d candidate(s)", candidate_idx+1);
      return;
    }

    Eigen::Vector3f P0 = best_candidate.P0;
    Eigen::Vector3f dir = best_candidate.dir;
    float t_min = best_candidate.t_min;
    float t_max = best_candidate.t_max;

    // Check length against expected board width and inlier ratio before publishing
    float current_len = t_max - t_min;
    double inlier_ratio = static_cast<double>(best_candidate.inlier_cnt) / merged_cloud->points.size();
    
    if (std::fabs(current_len - board_width_) > len_tol_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Reject line: length %.2f m not within tolerance of board width %.2f m",
                           current_len, board_width_);
      return;
    }

    if (inlier_ratio < min_inlier_ratio_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Reject line: inlier ratio %.2f below threshold %.2f",
                           inlier_ratio, min_inlier_ratio_);
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Line validated: length=%.3f m, inlier_ratio=%.3f", current_len, inlier_ratio);

    // Optional: extend endpoints to full board width if slightly shorter
    float extend = (board_width_ - current_len) / 2.0f;
    t_min -= extend;
    t_max += extend;

    Eigen::Vector3f P_min = P0 + dir * t_min;
    Eigen::Vector3f P_max = P0 + dir * t_max;
    Eigen::Vector3f center = 0.5f * (P_min + P_max);

    // Publish marker for visualization
    static int marker_seq = 0;
    visualization_msgs::msg::Marker line_marker;
    line_marker.header = msg->header;
    line_marker.ns = "board_line";
    line_marker.id = marker_seq++;
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
    
    // 检查marker话题的订阅者数量
    size_t subscriber_count = line_marker_pub_->get_subscription_count();
    RCLCPP_INFO(this->get_logger(), "Marker topic has %zu subscriber(s)", subscriber_count);
    
    line_marker_pub_->publish(line_marker);
    // RCLCPP_INFO(this->get_logger(), "Publishing marker: frame_id=%s, points=[(%.2f,%.2f), (%.2f,%.2f)]", 
    //             line_marker.header.frame_id.c_str(),
    //             line_marker.points[0].x, line_marker.points[0].y,
    //             line_marker.points[1].x, line_marker.points[1].y);

    // RCLCPP_INFO(this->get_logger(), "Marker published: id=%d Pmin(%.2f,%.2f) Pmax(%.2f,%.2f) len=%.2f", line_marker.id, P_min.x(), P_min.y(), P_max.x(), P_max.y(), current_len);

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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_passthrough_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_statistical_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_bilateral_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_gaussian_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_2d_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_clusters_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_selected_cluster_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr center_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // buffer of recent ROI clouds
  std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_buffer_;
  size_t history_size_;
  // Filter bounds
  double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
  double board_width_;
  double len_tol_;
  double min_inlier_ratio_;
  int max_candidates_;
  double ransac_dist_thresh_;
  std::string filter_type_;
  std::string advanced_filter_type_;
  bool comparison_mode_;
  int sor_mean_k_;
  double sor_stddev_mul_;
  double bilateral_sigma_s_;
  double bilateral_sigma_r_;
  double gaussian_radius_;
  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
  int max_clusters_;
  double max_cluster_dimension_;
  double min_cluster_dimension_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudFilterNode>());
  rclcpp::shutdown();
  return 0;
} 