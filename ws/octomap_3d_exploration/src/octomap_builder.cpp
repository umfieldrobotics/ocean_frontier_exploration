#include "octomap_3d_exploration/octomap_builder.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace octomap_3d_exploration
{

OctomapBuilder::OctomapBuilder()
: Node("octomap_builder")
{
  // Declare parameters  -- parameters.yaml will override these defaults
  this->declare_parameter("resolution", 0.2);
  this->declare_parameter("max_range", 15.0);
  this->declare_parameter("min_range", 0.5);
  this->declare_parameter("frame_id", "world");
  this->declare_parameter("robot_frame", "base_link");
  this->declare_parameter("publish_rate", 1.0);
  this->declare_parameter("max_depth", 16);
  this->declare_parameter("prob_hit", 0.7);
  this->declare_parameter("prob_miss", 0.4);
  this->declare_parameter("occupancy_threshold", 0.5);
  this->declare_parameter("clamping_threshold_min", 0.1192);
  this->declare_parameter("clamping_threshold_max", 0.971);
  this->declare_parameter("visualize_free_space", true);
  this->declare_parameter("visualize_unknown_space", false);
  this->declare_parameter("visualization_decimation", 10);

  // Get parameters
  resolution_ = this->get_parameter("resolution").as_double();
  max_range_ = this->get_parameter("max_range").as_double();
  min_range_ = this->get_parameter("min_range").as_double();
  frame_id_ = this->get_parameter("frame_id").as_string();
  robot_frame_ = this->get_parameter("robot_frame").as_string();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  max_depth_ = this->get_parameter("max_depth").as_int();
  prob_hit_ = this->get_parameter("prob_hit").as_double();
  prob_miss_ = this->get_parameter("prob_miss").as_double();
  occupancy_threshold_ = this->get_parameter("occupancy_threshold").as_double();
  clamping_threshold_min_ = this->get_parameter("clamping_threshold_min").as_double();
  clamping_threshold_max_ = this->get_parameter("clamping_threshold_max").as_double();
  visualize_free_space_ = this->get_parameter("visualize_free_space").as_bool();
  visualize_unknown_space_ = this->get_parameter("visualize_unknown_space").as_bool();
  visualization_decimation_ = this->get_parameter("visualization_decimation").as_int();

  // Initialize OctoMap
  octree_ = std::make_shared<octomap::OcTree>(resolution_);
  octree_->setProbHit(prob_hit_);
  octree_->setProbMiss(prob_miss_);
  octree_->setOccupancyThres(occupancy_threshold_);
  octree_->setClampingThresMin(clamping_threshold_min_);
  octree_->setClampingThresMax(clamping_threshold_max_);

  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscribers - using your topic: /UW_Camera_Stereo_pointcloud
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/UW_Camera_Stereo_pointcloud", 10,
    std::bind(&OctomapBuilder::pointCloudCallback, this, std::placeholders::_1));

  // Publishers
  octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>(
    "octomap_full", 10);
  octomap_binary_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>(
    "octomap_binary", 10);
  occupied_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "occupied_cells_vis", 10);
  free_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "free_cells_vis", 10);
  unknown_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "unknown_cells_vis", 10);

  // Timer for publishing
  publish_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_),
    std::bind(&OctomapBuilder::publishOctomap, this));

  RCLCPP_INFO(this->get_logger(), 
    "OctomapBuilder initialized:");
  RCLCPP_INFO(this->get_logger(), "  Resolution: %.2f m", resolution_);
  RCLCPP_INFO(this->get_logger(), "  Max range: %.2f m", max_range_);
  RCLCPP_INFO(this->get_logger(), "  Frame: %s", frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Subscribed to: /UW_Camera_Stereo_pointcloud");
}

OctomapBuilder::~OctomapBuilder()
{
}

bool OctomapBuilder::transformPointCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_in,
  pcl::PointCloud<pcl::PointXYZ>& cloud_out,
  geometry_msgs::msg::TransformStamped& transform)
{
  try {
    // Get transform from cloud frame to map frame
    // Use Time(0) to get the latest available transform instead of the point cloud timestamp
    // This avoids extrapolation errors when point cloud timestamps are slightly delayed
    transform = tf_buffer_->lookupTransform(
      frame_id_,
      cloud_in->header.frame_id,
      rclcpp::Time(0),
      rclcpp::Duration::from_seconds(0.1));

    // Convert ROS PointCloud2 to PCL
    pcl::PointCloud<pcl::PointXYZ> cloud_temp;
    pcl::fromROSMsg(*cloud_in, cloud_temp);

    // Transform point cloud
    cloud_out.clear();
    cloud_out.reserve(cloud_temp.size());

    for (const auto& point : cloud_temp.points) {
      // Skip invalid points
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }

      // Apply transform
      geometry_msgs::msg::PointStamped p_in, p_out;
      p_in.point.x = point.x;
      p_in.point.y = point.y;
      p_in.point.z = point.z;
      
      tf2::doTransform(p_in, p_out, transform);

      pcl::PointXYZ p;
      p.x = p_out.point.x;
      p.y = p_out.point.y;
      p.z = p_out.point.z;
      
      cloud_out.push_back(p);
    }

    return true;

  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Could not transform point cloud: %s", ex.what());
    return false;
  }
}

void OctomapBuilder::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Transform point cloud to map frame
  pcl::PointCloud<pcl::PointXYZ> cloud;
  geometry_msgs::msg::TransformStamped transform;
  
  if (!transformPointCloud(msg, cloud, transform)) {
    return;
  }

  if (cloud.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Received empty point cloud after transformation");
    return;
  }

  // Get sensor origin in map frame
  octomap::point3d sensor_origin(
    transform.transform.translation.x,
    transform.transform.translation.y,
    transform.transform.translation.z
  );

  // Build octomap point cloud with range filtering
  octomap::Pointcloud octo_cloud;
  octo_cloud.reserve(cloud.size());

  for (const auto& point : cloud.points) {
    double distance = std::sqrt(
      std::pow(point.x - sensor_origin.x(), 2) +
      std::pow(point.y - sensor_origin.y(), 2) +
      std::pow(point.z - sensor_origin.z(), 2)
    );

    // Filter by range
    if (distance >= min_range_ && distance <= max_range_) {
      octo_cloud.push_back(point.x, point.y, point.z);
    }
  }

  // Insert scan into octree (thread-safe)
  {
    std::lock_guard<std::mutex> lock(octree_mutex_);
    octree_->insertPointCloud(octo_cloud, sensor_origin, max_range_);

    // // CRITICAL: Update inner occupancy to propagate changes through tree
    // // This is needed for the map to persist and accumulate observations
    // octree_->updateInnerOccupancy();
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
    "Inserted %zu pts, octree now has %zu nodes",
    octo_cloud.size(), octree_->size());
}

void OctomapBuilder::publishOctomap()
{
  std::lock_guard<std::mutex> lock(octree_mutex_);

  if (octree_->size() == 0) {
    return;
  }

  // Update inner occupancy before publishing (done here at 2 Hz, not on every pointcloud)
  octree_->updateInnerOccupancy();

  auto stamp = this->now();

  // Publish full octomap
  octomap_msgs::msg::Octomap map_msg;
  map_msg.header.frame_id = frame_id_;
  map_msg.header.stamp = stamp;
  
  if (octomap_msgs::fullMapToMsg(*octree_, map_msg)) {
    octomap_pub_->publish(map_msg);
  }

  // Publish binary octomap
  octomap_msgs::msg::Octomap binary_msg;
  binary_msg.header.frame_id = frame_id_;
  binary_msg.header.stamp = stamp;
  
  if (octomap_msgs::binaryMapToMsg(*octree_, binary_msg)) {
    octomap_binary_pub_->publish(binary_msg);
  }

  // Publish visualization markers
  publishOccupiedMarkers();
  
  if (visualize_free_space_) {
    publishFreeMarkers();
  }
  
  if (visualize_unknown_space_) {
    publishUnknownMarkers();
  }

  RCLCPP_DEBUG(this->get_logger(), 
    "Published octomap with %zu nodes", octree_->size());
}

void OctomapBuilder::publishOccupiedMarkers()
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  
  marker.header.frame_id = frame_id_;
  marker.header.stamp = this->now();
  marker.ns = "occupied_cells";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = resolution_;
  marker.scale.y = resolution_;
  marker.scale.z = resolution_;
  marker.color.r = 0.8;
  marker.color.g = 0.2;
  marker.color.b = 0.2;
  marker.color.a = 0.9;

  // Iterate through octree
  for (octomap::OcTree::leaf_iterator it = octree_->begin_leafs();
       it != octree_->end_leafs(); ++it)
  {
    if (octree_->isNodeOccupied(*it)) {
      geometry_msgs::msg::Point p;
      p.x = it.getX();
      p.y = it.getY();
      p.z = it.getZ();
      marker.points.push_back(p);
    }
  }

  // Always publish (even if empty) so RViz clears stale markers
  marker_array.markers.push_back(marker);
  occupied_marker_pub_->publish(marker_array);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "Octomap: %zu occupied voxels, %zu total nodes", marker.points.size(), octree_->size());
}

void OctomapBuilder::publishFreeMarkers()
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  
  marker.header.frame_id = frame_id_;
  marker.header.stamp = this->now();
  marker.ns = "free_cells";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = resolution_;
  marker.scale.y = resolution_;
  marker.scale.z = resolution_;
  marker.color.r = 0.2;
  marker.color.g = 0.8;
  marker.color.b = 0.2;
  marker.color.a = 0.3;

  // Sample free cells to avoid too many markers
  int count = 0;
  for (octomap::OcTree::leaf_iterator it = octree_->begin_leafs();
       it != octree_->end_leafs(); ++it)
  {
    if (!octree_->isNodeOccupied(*it)) {
      if (count++ % visualization_decimation_ == 0) {
        geometry_msgs::msg::Point p;
        p.x = it.getX();
        p.y = it.getY();
        p.z = it.getZ();
        marker.points.push_back(p);
      }
    }
  }

  // Always publish (even if empty) so RViz clears stale markers
  marker_array.markers.push_back(marker);
  free_marker_pub_->publish(marker_array);
}

void OctomapBuilder::publishUnknownMarkers()
{
  // This will be used later for frontier visualization
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  
  marker.header.frame_id = frame_id_;
  marker.header.stamp = this->now();
  marker.ns = "unknown_cells";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = resolution_;
  marker.scale.y = resolution_;
  marker.scale.z = resolution_;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.8;
  marker.color.a = 0.5;

  // Note: Unknown cells are not explicitly stored in OctoMap
  // They are inferred during frontier detection
  // This is a placeholder for future frontier visualization

  unknown_marker_pub_->publish(marker_array);
}

}  // namespace octomap_3d_exploration

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<octomap_3d_exploration::OctomapBuilder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}