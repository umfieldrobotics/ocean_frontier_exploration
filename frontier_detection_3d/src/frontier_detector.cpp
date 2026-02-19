#include "frontier_detection_3d/frontier_detector.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <algorithm>

namespace frontier_detection_3d
{

FrontierDetector::FrontierDetector()
: Node("frontier_detector")
{
  // Declare parameters
  this->declare_parameter("detection_rate", 1.0);
  this->declare_parameter("min_frontier_size", 5.0);
  this->declare_parameter("cluster_radius", 1.0);
  this->declare_parameter("information_radius", 2.0);
  this->declare_parameter("lambda_distance", 0.1);
  this->declare_parameter("map_frame", "UW_camera_world");
  this->declare_parameter("robot_frame", "base_link");
  this->declare_parameter("refinement_depth", 14);
  
  // Get parameters
  detection_rate_ = this->get_parameter("detection_rate").as_double();
  min_frontier_size_ = this->get_parameter("min_frontier_size").as_double();
  cluster_radius_ = this->get_parameter("cluster_radius").as_double();
  information_radius_ = this->get_parameter("information_radius").as_double();
  lambda_distance_ = this->get_parameter("lambda_distance").as_double();
  map_frame_ = this->get_parameter("map_frame").as_string();
  robot_frame_ = this->get_parameter("robot_frame").as_string();
  refinement_depth_ = this->get_parameter("refinement_depth").as_int();
  
  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Subscribers
  octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
    "/octomap_binary", 10,
    std::bind(&FrontierDetector::octomapCallback, this, std::placeholders::_1));
  
  // Publishers
  goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "exploration_goal", 10);
  frontier_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "frontier_markers", 10);
  best_frontier_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "best_frontier_marker", 10);
  
  // Timer for periodic frontier detection
  detection_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / detection_rate_),
    std::bind(&FrontierDetector::detectFrontiers, this));
  
  RCLCPP_INFO(this->get_logger(), "FrontierDetector initialized");
  RCLCPP_INFO(this->get_logger(), "  Detection rate: %.2f Hz", detection_rate_);
  RCLCPP_INFO(this->get_logger(), "  Cluster radius: %.2f m", cluster_radius_);
  RCLCPP_INFO(this->get_logger(), "  Map frame: %s", map_frame_.c_str());
}

FrontierDetector::~FrontierDetector()
{
}

void FrontierDetector::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(octree_mutex_);
  
  // Convert message to octree
  octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(*msg);
  if (!abstract_tree) {
    RCLCPP_WARN(this->get_logger(), "Failed to convert octomap message");
    return;
  }
  
  // Cast to OcTree
  octree_.reset(dynamic_cast<octomap::OcTree*>(abstract_tree));
  
  if (!octree_) {
    RCLCPP_WARN(this->get_logger(), "Failed to cast to OcTree");
    return;
  }
  
  RCLCPP_DEBUG(this->get_logger(), "Received octomap with %zu nodes", octree_->size());
}

void FrontierDetector::detectFrontiers()
{
  std::lock_guard<std::mutex> lock(octree_mutex_);

  if (!octree_ || octree_->size() == 0) {
    RCLCPP_DEBUG(this->get_logger(), "No octree available yet");
    return;
  }

  frontier_voxels_.clear();

  // Iterate through all leaf nodes in the octree
  for (octomap::OcTree::leaf_iterator it = octree_->begin_leafs(refinement_depth_);
       it != octree_->end_leafs(); ++it)
  {
    // Only consider FREE voxels
    if (!octree_->isNodeOccupied(*it)) {
      octomap::point3d position = it.getCoordinate();

      // A frontier is a free voxel adjacent to unknown space
      if (hasUnknownNeighbor(position)) {
        FrontierVoxel frontier;
        frontier.position = position;
        frontier.information_gain = 0.0;
        frontier_voxels_.push_back(frontier);
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "Found %zu frontier voxels", frontier_voxels_.size());

  if (frontier_voxels_.empty()) {
    RCLCPP_INFO(this->get_logger(), "No frontiers found - exploration complete!");
    return;
  }

  // Find the nearest frontier voxel to the robot - no clustering needed
  octomap::point3d robot_pos = getRobotPosition();
  double min_dist = std::numeric_limits<double>::infinity();
  const FrontierVoxel* nearest = nullptr;

  for (const auto& fv : frontier_voxels_) {
    double d = calculateDistance(robot_pos, fv.position);
    if (d < min_dist) {
      min_dist = d;
      nearest = &fv;
    }
  }

  if (!nearest) return;

  // Set best_frontier_ as a single-voxel "cluster" at the nearest frontier
  best_frontier_.centroid = nearest->position;
  best_frontier_.voxels = {*nearest};
  best_frontier_.score = 1.0;
  frontier_clusters_ = {best_frontier_};

  RCLCPP_INFO(this->get_logger(),
    "Nearest frontier at (%.2f, %.2f, %.2f), dist=%.2fm",
    nearest->position.x(), nearest->position.y(), nearest->position.z(), min_dist);

  // Publish results
  publishFrontierMarkers();
  publishBestFrontier();
  publishExplorationGoal();
}

bool FrontierDetector::hasUnknownNeighbor(const octomap::point3d& position)
{
  double resolution = octree_->getResolution();
  
  // Check 6-connected neighbors (±X, ±Y, ±Z)
  std::vector<octomap::point3d> neighbors = {
    octomap::point3d(position.x() + resolution, position.y(), position.z()),
    octomap::point3d(position.x() - resolution, position.y(), position.z()),
    octomap::point3d(position.x(), position.y() + resolution, position.z()),
    octomap::point3d(position.x(), position.y() - resolution, position.z()),
    octomap::point3d(position.x(), position.y(), position.z() + resolution),
    octomap::point3d(position.x(), position.y(), position.z() - resolution)
  };
  
  for (const auto& neighbor : neighbors) {
    octomap::OcTreeNode* node = octree_->search(neighbor);
    if (node == nullptr) {
      // Unknown voxel found!
      return true;
    }
  }
  
  return false;
}

double FrontierDetector::calculateInformationGain(const octomap::point3d& position)
{
  // Count unknown voxels in a cube around this position
  double half_size = information_radius_ / 2.0;
  
  octomap::point3d min_bound(
    position.x() - half_size,
    position.y() - half_size,
    position.z() - half_size
  );
  
  octomap::point3d max_bound(
    position.x() + half_size,
    position.y() + half_size,
    position.z() + half_size
  );
  
  for (octomap::OcTree::leaf_bbx_iterator it = octree_->begin_leafs_bbx(min_bound, max_bound);
       it != octree_->end_leafs_bbx(); ++it)
  {
    // Unknown voxels are not in the tree
    // So we estimate based on volume
  }
  
  // Simple estimate: volume of exploration cube
  double volume = information_radius_ * information_radius_ * information_radius_;
  double voxel_volume = std::pow(octree_->getResolution(), 3);
  
  return volume / voxel_volume;  // Estimated unknown voxels
}

void FrontierDetector::clusterFrontiers()
{
  frontier_clusters_.clear();
  
  if (frontier_voxels_.empty()) {
    return;
  }
  
  // Simple clustering: group nearby frontiers
  std::vector<bool> clustered(frontier_voxels_.size(), false);
  
  for (size_t i = 0; i < frontier_voxels_.size(); ++i) {
    if (clustered[i]) continue;
    
    FrontierCluster cluster;
    cluster.voxels.push_back(frontier_voxels_[i]);
    cluster.total_information_gain = frontier_voxels_[i].information_gain;
    clustered[i] = true;
    
    // Find nearby frontiers
    for (size_t j = i + 1; j < frontier_voxels_.size(); ++j) {
      if (clustered[j]) continue;
      
      double dist = calculateDistance(
        frontier_voxels_[i].position,
        frontier_voxels_[j].position
      );
      
      if (dist < cluster_radius_) {
        cluster.voxels.push_back(frontier_voxels_[j]);
        cluster.total_information_gain += frontier_voxels_[j].information_gain;
        clustered[j] = true;
      }
    }
    
    // Calculate centroid
    octomap::point3d sum(0, 0, 0);
    for (const auto& voxel : cluster.voxels) {
      sum += voxel.position;
    }
    cluster.centroid = sum * (1.0 / cluster.voxels.size());
    
    // Only keep clusters with minimum size
    if (cluster.voxels.size() >= static_cast<size_t>(min_frontier_size_)) {
      frontier_clusters_.push_back(cluster);
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "Clustered into %zu frontier clusters", 
    frontier_clusters_.size());
}

void FrontierDetector::selectBestFrontier()
{
  if (frontier_clusters_.empty()) {
    return;
  }
  
  octomap::point3d robot_pos = getRobotPosition();
  
  // Score each cluster
  double best_score = -std::numeric_limits<double>::infinity();
  
  for (auto& cluster : frontier_clusters_) {
    cluster.distance_to_robot = calculateDistance(robot_pos, cluster.centroid);
    
    // Scoring function: information_gain * exp(-lambda * distance)
    cluster.score = cluster.total_information_gain * 
                    std::exp(-lambda_distance_ * cluster.distance_to_robot);
    
    if (cluster.score > best_score) {
      best_score = cluster.score;
      best_frontier_ = cluster;
    }
  }
  
  RCLCPP_INFO(this->get_logger(), 
    "Best frontier at (%.2f, %.2f, %.2f) with score %.2f",
    best_frontier_.centroid.x(), best_frontier_.centroid.y(), best_frontier_.centroid.z(),
    best_frontier_.score);
}

octomap::point3d FrontierDetector::getRobotPosition()
{
  try {
    auto transform = tf_buffer_->lookupTransform(
      map_frame_,
      robot_frame_,
      rclcpp::Time(0),
      rclcpp::Duration::from_seconds(0.1)
    );
    
    return octomap::point3d(
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z
    );
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get robot position: %s", ex.what());
    return octomap::point3d(0, 0, 0);
  }
}

double FrontierDetector::calculateDistance(
  const octomap::point3d& p1, 
  const octomap::point3d& p2)
{
  return std::sqrt(
    std::pow(p2.x() - p1.x(), 2) +
    std::pow(p2.y() - p1.y(), 2) +
    std::pow(p2.z() - p1.z(), 2)
  );
}

void FrontierDetector::publishFrontierMarkers()
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  
  marker.header.frame_id = map_frame_;
  marker.header.stamp = this->now();
  marker.ns = "frontiers";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = octree_->getResolution();
  marker.scale.y = octree_->getResolution();
  marker.scale.z = octree_->getResolution();
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 0.6;
  
  for (const auto& voxel : frontier_voxels_) {
    geometry_msgs::msg::Point p;
    p.x = voxel.position.x();
    p.y = voxel.position.y();
    p.z = voxel.position.z();
    marker.points.push_back(p);
  }
  
  if (!marker.points.empty()) {
    marker_array.markers.push_back(marker);
    frontier_marker_pub_->publish(marker_array);
  }
}

void FrontierDetector::publishBestFrontier()
{
  if (frontier_clusters_.empty()) {
    return;
  }
  
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  
  marker.header.frame_id = map_frame_;
  marker.header.stamp = this->now();
  marker.ns = "best_frontier";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = best_frontier_.centroid.x();
  marker.pose.position.y = best_frontier_.centroid.y();
  marker.pose.position.z = best_frontier_.centroid.z();
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  
  marker_array.markers.push_back(marker);
  best_frontier_marker_pub_->publish(marker_array);
}

void FrontierDetector::publishExplorationGoal()
{
  if (frontier_clusters_.empty()) {
    return;
  }

  // Navigate directly to the best frontier centroid
  // The best frontier is already the single highest-scoring one from selectBestFrontier()
  octomap::point3d robot_pos = getRobotPosition();
  octomap::point3d frontier_pos = best_frontier_.centroid;
  octomap::point3d direction = robot_pos - frontier_pos;
  double distance = direction.norm();

  // Place goal slightly before the frontier (toward robot) to avoid hitting walls
  octomap::point3d goal_pos;
  if (distance > 0.01) {
    double offset = std::max(0.3, std::min(distance * 0.4, 0.5));
    goal_pos = frontier_pos + direction * (offset / distance);
  } else {
    goal_pos = frontier_pos;
  }

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = map_frame_;
  goal.header.stamp = this->now();
  goal.pose.position.x = goal_pos.x();
  goal.pose.position.y = goal_pos.y();
  goal.pose.position.z = goal_pos.z();
  goal.pose.orientation.w = 1.0;

  goal_pub_->publish(goal);

  RCLCPP_INFO(this->get_logger(),
    "Goal: frontier=(%.2f,%.2f,%.2f) -> goal=(%.2f,%.2f,%.2f) dist=%.2fm",
    frontier_pos.x(), frontier_pos.y(), frontier_pos.z(),
    goal_pos.x(), goal_pos.y(), goal_pos.z(), distance);
}

}  // namespace frontier_detection_3d

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<frontier_detection_3d::FrontierDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}