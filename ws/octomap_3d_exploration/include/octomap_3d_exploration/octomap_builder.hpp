#ifndef OCTOMAP_3D_EXPLORATION__OCTOMAP_BUILDER_HPP_
#define OCTOMAP_3D_EXPLORATION__OCTOMAP_BUILDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace octomap_3d_exploration
{

class OctomapBuilder : public rclcpp::Node
{
public:
  OctomapBuilder();
  ~OctomapBuilder();

  // Expose octree for frontier detection
  std::shared_ptr<octomap::OcTree> getOctree() { return octree_; }

private:
  // Callback functions
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publishOctomap();
  void publishOccupiedMarkers();
  void publishFreeMarkers();
  void publishUnknownMarkers();

  // Helper functions
  bool transformPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_in,
                          pcl::PointCloud<pcl::PointXYZ>& cloud_out,
                          geometry_msgs::msg::TransformStamped& transform);

  // ROS2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_binary_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr occupied_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr free_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr unknown_marker_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // OctoMap
  std::shared_ptr<octomap::OcTree> octree_;
  std::mutex octree_mutex_;

  // Parameters
  double resolution_;
  double max_range_;
  double min_range_;
  std::string frame_id_;
  std::string robot_frame_;
  double publish_rate_;
  int max_depth_;
  
  // Probabilities
  double prob_hit_;
  double prob_miss_;
  double occupancy_threshold_;
  double clamping_threshold_min_;
  double clamping_threshold_max_;

  // Visualization
  bool visualize_free_space_;
  bool visualize_unknown_space_;
  int visualization_decimation_;
};

}  // namespace octomap_3d_exploration

#endif  // OCTOMAP_3D_EXPLORATION__OCTOMAP_BUILDER_HPP_