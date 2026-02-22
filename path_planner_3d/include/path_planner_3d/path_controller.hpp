#ifndef PATH_PLANNER_3D__PATH_CONTROLLER_HPP_
#define PATH_PLANNER_3D__PATH_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <memory>
#include <vector>
#include <cmath>

namespace path_planner_3d
{

struct Waypoint
{
  double x, y, z;
  Waypoint(double x_ = 0, double y_ = 0, double z_ = 0) : x(x_), y(y_), z(z_) {}
};

class PathController : public rclcpp::Node
{
public:
  PathController();
  ~PathController() = default;

private:
  // Callbacks
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void controlLoop();

  // Navigation
  Waypoint getRobotPosition();
  double getYawToWaypoint(const Waypoint& robot_pos, const Waypoint& target);
  void updateCurrentWaypoint();
  geometry_msgs::msg::Twist computeVelocityCommand();

  // Utilities
  double distance3D(const Waypoint& a, const Waypoint& b);
  double normalizeAngle(double angle);

  // ROS2 interfaces
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Path following state
  std::vector<Waypoint> current_path_;
  size_t current_waypoint_idx_;
  bool has_path_;
  bool path_completed_;

  // Parameters
  std::string map_frame_;
  std::string robot_frame_;
  double control_frequency_;
  double waypoint_tolerance_;
  double goal_tolerance_;
  double max_linear_velocity_;
  double max_angular_velocity_;
  double k_linear_;   // Linear velocity gain
  double k_angular_;  // Angular velocity gain
};

}  // namespace path_planner_3d

#endif  // PATH_PLANNER_3D__PATH_CONTROLLER_HPP_
