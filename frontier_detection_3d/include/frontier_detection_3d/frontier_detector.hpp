#ifndef FRONTIER_DETECTION_3D__FRONTIER_DETECTOR_HPP_
#define FRONTIER_DETECTION_3D__FRONTIER_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <vector>
#include <unordered_set>


namespace frontier_detection_3d
{

struct FrontierVoxel
{
  octomap::point3d position;
  double information_gain;
};

struct FrontierCluster
{
  octomap::point3d centroid;
  std::vector<FrontierVoxel> voxels;
  double total_information_gain;
  double distance_to_robot;
  double score;
};

class FrontierDetector : public rclcpp::Node
{
public:
  FrontierDetector();
  ~FrontierDetector();

private:
  // Callback functions
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
  void detectFrontiers();
  void clusterFrontiers();
  void selectBestFrontier();
  
  // Frontier detection
  bool isFrontierVoxel(const octomap::point3d& position);
  bool hasUnknownNeighbor(const octomap::point3d& position);
  double calculateInformationGain(const octomap::point3d& position);
  
  // Clustering
  void meanShiftClustering(const std::vector<FrontierVoxel>& frontiers);
  
  // Publishing
  void publishFrontierMarkers();
  void publishBestFrontier();
  void publishExplorationGoal();
  
  // Utility
  octomap::point3d getRobotPosition();
  double calculateDistance(const octomap::point3d& p1, const octomap::point3d& p2);
  
  // ROS2 interfaces
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr best_frontier_marker_pub_;
  rclcpp::TimerBase::SharedPtr detection_timer_;
  
  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // OctoMap
  std::shared_ptr<octomap::OcTree> octree_;
  std::mutex octree_mutex_;
  
  // Frontier data
  std::vector<FrontierVoxel> frontier_voxels_;
  std::vector<FrontierCluster> frontier_clusters_;
  FrontierCluster best_frontier_;
  
  // Parameters
  double detection_rate_;
  double min_frontier_size_;
  double cluster_radius_;
  double information_radius_;
  double lambda_distance_;
  std::string map_frame_;
  std::string robot_frame_;
  int refinement_depth_;
};

}  // namespace frontier_detection_3d

#endif  // FRONTIER_DETECTION_3D__FRONTIER_DETECTOR_HPP_