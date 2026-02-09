#ifndef PATH_PLANNER_3D__PATH_PLANNER_HPP_
#define PATH_PLANNER_3D__PATH_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <memory>
#include <mutex>
#include <vector>

namespace path_planner_3d
{

struct Point3D
{
  double x, y, z;
  Point3D(double x_ = 0, double y_ = 0, double z_ = 0) : x(x_), y(y_), z(z_) {}
};

class PathPlanner : public rclcpp::Node
{
public:
  PathPlanner();
  ~PathPlanner() = default;

private:
  // Callbacks
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);

  // Path planning
  bool planPath(const Point3D& start, const Point3D& goal, std::vector<Point3D>& path);
  bool isStateValid(const Point3D& point);
  bool isInCollision(const Point3D& point);
  Point3D getRobotPosition();

  // Path smoothing
  void smoothPath(std::vector<Point3D>& path);

  // Publishing
  void publishPath(const std::vector<Point3D>& path);
  void publishPathVisualization(const std::vector<Point3D>& path);

  // ROS2 interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_vis_pub_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Octomap
  std::shared_ptr<octomap::OcTree> octree_;
  std::mutex octree_mutex_;

  // Parameters
  std::string map_frame_;
  std::string robot_frame_;
  double planning_timeout_;
  double collision_check_resolution_;
  double robot_radius_;
  double max_planning_range_;
  bool use_rrt_star_;

  // State
  geometry_msgs::msg::PoseStamped current_goal_;
  bool has_goal_;
  bool has_octree_;
};

}  // namespace path_planner_3d

#endif  // PATH_PLANNER_3D__PATH_PLANNER_HPP_
