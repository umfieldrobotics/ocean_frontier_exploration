#include "path_planner_3d/path_controller.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace path_planner_3d
{

PathController::PathController()
: Node("path_controller"),
  current_waypoint_idx_(0),
  has_path_(false),
  path_completed_(true)
{
  // Declare parameters
  this->declare_parameter("map_frame", "UW_camera_world");
  this->declare_parameter("robot_frame", "base_link");
  this->declare_parameter("control_frequency", 10.0);
  this->declare_parameter("waypoint_tolerance", 0.5);
  this->declare_parameter("goal_tolerance", 0.3);
  this->declare_parameter("max_linear_velocity", 1.0);
  this->declare_parameter("max_angular_velocity", 0.5);
  this->declare_parameter("k_linear", 0.5);
  this->declare_parameter("k_angular", 1.0);

  // Get parameters
  map_frame_ = this->get_parameter("map_frame").as_string();
  robot_frame_ = this->get_parameter("robot_frame").as_string();
  control_frequency_ = this->get_parameter("control_frequency").as_double();
  waypoint_tolerance_ = this->get_parameter("waypoint_tolerance").as_double();
  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
  max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
  max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
  k_linear_ = this->get_parameter("k_linear").as_double();
  k_angular_ = this->get_parameter("k_angular").as_double();

  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create subscribers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/planned_path", 10,
    std::bind(&PathController::pathCallback, this, std::placeholders::_1));

  // Create publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_target", 10);

  // Create control timer
  auto period = std::chrono::duration<double>(1.0 / control_frequency_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&PathController::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "PathController initialized");
  RCLCPP_INFO(this->get_logger(), "  Control frequency: %.1f Hz", control_frequency_);
  RCLCPP_INFO(this->get_logger(), "  Max linear velocity: %.2f m/s", max_linear_velocity_);
  RCLCPP_INFO(this->get_logger(), "  Max angular velocity: %.2f rad/s", max_angular_velocity_);
  RCLCPP_INFO(this->get_logger(), "  Waypoint tolerance: %.2f m", waypoint_tolerance_);
}

void PathController::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty path");
    return;
  }

  // Convert path to waypoints
  current_path_.clear();
  for (const auto& pose : msg->poses) {
    current_path_.emplace_back(
      pose.pose.position.x,
      pose.pose.position.y,
      pose.pose.position.z);
  }

  current_waypoint_idx_ = 0;
  has_path_ = true;
  path_completed_ = false;

  RCLCPP_INFO(this->get_logger(), "Received new path with %zu waypoints", current_path_.size());
}

Waypoint PathController::getRobotPosition()
{
  try {
    auto transform = tf_buffer_->lookupTransform(
      map_frame_,
      robot_frame_,
      rclcpp::Time(0),
      rclcpp::Duration::from_seconds(0.1));

    return Waypoint(
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Could not get robot transform: %s", ex.what());
    return Waypoint(0, 0, 0);
  }
}

double PathController::distance3D(const Waypoint& a, const Waypoint& b)
{
  return std::sqrt(
    std::pow(b.x - a.x, 2) +
    std::pow(b.y - a.y, 2) +
    std::pow(b.z - a.z, 2));
}

double PathController::normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double PathController::getYawToWaypoint(const Waypoint& robot_pos, const Waypoint& target)
{
  return std::atan2(target.y - robot_pos.y, target.x - robot_pos.x);
}

void PathController::updateCurrentWaypoint()
{
  if (!has_path_ || path_completed_) return;

  Waypoint robot_pos = getRobotPosition();
  Waypoint& current_waypoint = current_path_[current_waypoint_idx_];

  double distance = distance3D(robot_pos, current_waypoint);

  // Check if we reached current waypoint
  if (distance < waypoint_tolerance_) {
    current_waypoint_idx_++;

    if (current_waypoint_idx_ >= current_path_.size()) {
      // Reached end of path
      RCLCPP_INFO(this->get_logger(), "Path completed!");
      path_completed_ = true;
      has_path_ = false;

      // Stop robot
      geometry_msgs::msg::Twist stop_cmd;
      cmd_vel_pub_->publish(stop_cmd);
    } else {
      RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu/%zu",
        current_waypoint_idx_, current_path_.size());
    }
  }
}

geometry_msgs::msg::Twist PathController::computeVelocityCommand()
{
  geometry_msgs::msg::Twist cmd_vel;

  if (!has_path_ || path_completed_) {
    return cmd_vel;  // Zero velocity
  }

  Waypoint robot_pos = getRobotPosition();
  Waypoint& target = current_path_[current_waypoint_idx_];

  // Compute 3D distance and direction
  double dx = target.x - robot_pos.x;
  double dy = target.y - robot_pos.y;
  double dz = target.z - robot_pos.z;
  double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

  if (distance < 0.01) {
    return cmd_vel;  // Already at target
  }

  // Compute desired velocities (proportional control)
  double vx = k_linear_ * dx;
  double vy = k_linear_ * dy;
  double vz = k_linear_ * dz;

  // Compute velocity magnitude
  double v_mag = std::sqrt(vx*vx + vy*vy + vz*vz);

  // Limit to max velocity
  if (v_mag > max_linear_velocity_) {
    double scale = max_linear_velocity_ / v_mag;
    vx *= scale;
    vy *= scale;
    vz *= scale;
  }

  // For underwater 6-DOF, we publish velocity in robot frame
  // Isaac Sim typically expects cmd_vel in local frame
  cmd_vel.linear.x = vx;
  cmd_vel.linear.y = vy;
  cmd_vel.linear.z = vz;

  // Simple yaw control to face target
  double desired_yaw = getYawToWaypoint(robot_pos, target);

  // Get current yaw from TF
  try {
    auto transform = tf_buffer_->lookupTransform(
      map_frame_,
      robot_frame_,
      rclcpp::Time(0),
      rclcpp::Duration::from_seconds(0.1));

    tf2::Quaternion q(
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z,
      transform.transform.rotation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double yaw_error = normalizeAngle(desired_yaw - yaw);
    cmd_vel.angular.z = k_angular_ * yaw_error;

    // Limit angular velocity
    if (cmd_vel.angular.z > max_angular_velocity_) {
      cmd_vel.angular.z = max_angular_velocity_;
    } else if (cmd_vel.angular.z < -max_angular_velocity_) {
      cmd_vel.angular.z = -max_angular_velocity_;
    }
  } catch (tf2::TransformException& ex) {
    // Can't get orientation, skip angular control
  }

  return cmd_vel;
}

void PathController::controlLoop()
{
  if (!has_path_ || path_completed_) {
    return;
  }

  // Update waypoint
  updateCurrentWaypoint();

  if (path_completed_) {
    return;
  }

  // Compute and publish velocity command
  geometry_msgs::msg::Twist cmd_vel = computeVelocityCommand();
  cmd_vel_pub_->publish(cmd_vel);

  // Publish current target for visualization
  geometry_msgs::msg::PoseStamped target_msg;
  target_msg.header.frame_id = map_frame_;
  target_msg.header.stamp = this->now();
  target_msg.pose.position.x = current_path_[current_waypoint_idx_].x;
  target_msg.pose.position.y = current_path_[current_waypoint_idx_].y;
  target_msg.pose.position.z = current_path_[current_waypoint_idx_].z;
  target_msg.pose.orientation.w = 1.0;
  target_pub_->publish(target_msg);
}

}  // namespace path_planner_3d

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<path_planner_3d::PathController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
