#include "path_planner_3d/path_planner.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <random>
#include <algorithm>

namespace path_planner_3d
{

// RRT Node structure
struct RRTNode
{
  Point3D position;
  int parent_idx;
  double cost;

  RRTNode(const Point3D& pos, int parent = -1, double c = 0.0)
    : position(pos), parent_idx(parent), cost(c) {}
};

PathPlanner::PathPlanner()
: Node("path_planner"),
  has_goal_(false),
  has_octree_(false)
{
  // Declare parameters
  this->declare_parameter("map_frame", "UW_camera_world");
  this->declare_parameter("robot_frame", "base_link");
  this->declare_parameter("planning_timeout", 5.0);
  this->declare_parameter("collision_check_resolution", 0.2);
  this->declare_parameter("robot_radius", 0.5);
  this->declare_parameter("max_planning_range", 50.0);
  this->declare_parameter("use_rrt_star", true);

  // Get parameters
  map_frame_ = this->get_parameter("map_frame").as_string();
  robot_frame_ = this->get_parameter("robot_frame").as_string();
  planning_timeout_ = this->get_parameter("planning_timeout").as_double();
  collision_check_resolution_ = this->get_parameter("collision_check_resolution").as_double();
  robot_radius_ = this->get_parameter("robot_radius").as_double();
  max_planning_range_ = this->get_parameter("max_planning_range").as_double();
  use_rrt_star_ = this->get_parameter("use_rrt_star").as_bool();

  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create subscribers
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/exploration_goal", 10,
    std::bind(&PathPlanner::goalCallback, this, std::placeholders::_1));

  octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
    "/octomap_binary", 10,
    std::bind(&PathPlanner::octomapCallback, this, std::placeholders::_1));

  // Create publishers
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
  path_vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/path_visualization", 10);

  RCLCPP_INFO(this->get_logger(), "PathPlanner initialized");
  RCLCPP_INFO(this->get_logger(), "  Map frame: %s", map_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Robot frame: %s", robot_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Planning timeout: %.1f s", planning_timeout_);
  RCLCPP_INFO(this->get_logger(), "  Robot radius: %.2f m", robot_radius_);
}

void PathPlanner::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(octree_mutex_);

  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
  octree_.reset(dynamic_cast<octomap::OcTree*>(tree));

  if (!octree_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to convert octomap message");
    return;
  }

  if (!has_octree_) {
    RCLCPP_INFO(this->get_logger(), "Received first octomap");
    has_octree_ = true;
  }
}

void PathPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!has_octree_) {
    RCLCPP_WARN(this->get_logger(), "No octomap available yet, cannot plan");
    return;
  }

  current_goal_ = *msg;
  has_goal_ = true;

  RCLCPP_INFO(this->get_logger(), "Received new goal: (%.2f, %.2f, %.2f)",
    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

  // Get robot position
  Point3D start = getRobotPosition();
  Point3D goal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

  RCLCPP_INFO(this->get_logger(), "Planning from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)",
    start.x, start.y, start.z, goal.x, goal.y, goal.z);

  // Plan path
  std::vector<Point3D> path;
  if (planPath(start, goal, path)) {
    RCLCPP_INFO(this->get_logger(), "Path found with %zu waypoints", path.size());

    // Smooth path
    smoothPath(path);
    RCLCPP_INFO(this->get_logger(), "Smoothed to %zu waypoints", path.size());

    // Publish path
    publishPath(path);
    publishPathVisualization(path);
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to find path to goal");
  }
}

Point3D PathPlanner::getRobotPosition()
{
  try {
    auto transform = tf_buffer_->lookupTransform(
      map_frame_,
      robot_frame_,
      rclcpp::Time(0),
      rclcpp::Duration::from_seconds(0.5));

    return Point3D(
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get robot transform: %s", ex.what());
    return Point3D(0, 0, 0);
  }
}

bool PathPlanner::isInCollision(const Point3D& point)
{
  std::lock_guard<std::mutex> lock(octree_mutex_);

  if (!octree_) return false;

  // Check if point is occupied in octree
  octomap::point3d query(point.x, point.y, point.z);
  octomap::OcTreeNode* node = octree_->search(query);

  if (node) {
    return octree_->isNodeOccupied(node);
  }

  // Unknown space - treat as free for planning
  return false;
}

bool PathPlanner::isStateValid(const Point3D& point)
{
  // Check collision with robot radius
  const int num_checks = 8;
  const double angles[] = {0, M_PI/4, M_PI/2, 3*M_PI/4, M_PI, 5*M_PI/4, 3*M_PI/2, 7*M_PI/4};

  for (int i = 0; i < num_checks; ++i) {
    Point3D check_point(
      point.x + robot_radius_ * std::cos(angles[i]),
      point.y + robot_radius_ * std::sin(angles[i]),
      point.z);

    if (isInCollision(check_point)) {
      return false;
    }
  }

  // Check vertical clearance
  Point3D above(point.x, point.y, point.z + robot_radius_);
  Point3D below(point.x, point.y, point.z - robot_radius_);

  if (isInCollision(above) || isInCollision(below)) {
    return false;
  }

  return true;
}

bool PathPlanner::planPath(const Point3D& start, const Point3D& goal, std::vector<Point3D>& path)
{
  // Check if start and goal are valid
  if (!isStateValid(start)) {
    RCLCPP_ERROR(this->get_logger(), "Start position is in collision!");
    return false;
  }

  if (!isStateValid(goal)) {
    RCLCPP_WARN(this->get_logger(), "Goal position is in collision or too close to obstacles");
    return false;
  }

  // RRT* parameters
  const int max_iterations = 5000;
  const double step_size = 1.0;  // meters
  const double goal_tolerance = 0.5;  // meters
  const double rewire_radius = 3.0;  // for RRT*

  // Initialize RRT
  std::vector<RRTNode> tree;
  tree.push_back(RRTNode(start, -1, 0.0));

  // Random number generation
  std::random_device rd;
  std::mt19937 gen(rd());

  // Get bounds from octomap
  double min_x = start.x - max_planning_range_;
  double max_x = start.x + max_planning_range_;
  double min_y = start.y - max_planning_range_;
  double max_y = start.y + max_planning_range_;
  double min_z = start.z - max_planning_range_;
  double max_z = start.z + max_planning_range_;

  std::uniform_real_distribution<> dis_x(min_x, max_x);
  std::uniform_real_distribution<> dis_y(min_y, max_y);
  std::uniform_real_distribution<> dis_z(min_z, max_z);
  std::uniform_real_distribution<> dis_goal(0.0, 1.0);

  auto distance = [](const Point3D& a, const Point3D& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
  };

  int goal_idx = -1;
  auto start_time = this->now();

  RCLCPP_INFO(this->get_logger(), "Starting RRT* planning...");

  for (int iter = 0; iter < max_iterations; ++iter) {
    // Check timeout
    if ((this->now() - start_time).seconds() > planning_timeout_) {
      RCLCPP_WARN(this->get_logger(), "Planning timeout after %d iterations", iter);
      break;
    }

    // Sample random point (with goal bias)
    Point3D rand_point;
    if (dis_goal(gen) < 0.1) {  // 10% goal bias
      rand_point = goal;
    } else {
      rand_point = Point3D(dis_x(gen), dis_y(gen), dis_z(gen));
    }

    // Find nearest node
    int nearest_idx = 0;
    double min_dist = distance(tree[0].position, rand_point);
    for (size_t i = 1; i < tree.size(); ++i) {
      double dist = distance(tree[i].position, rand_point);
      if (dist < min_dist) {
        min_dist = dist;
        nearest_idx = i;
      }
    }

    // Steer towards random point
    Point3D nearest_pos = tree[nearest_idx].position;
    Point3D direction;
    direction.x = rand_point.x - nearest_pos.x;
    direction.y = rand_point.y - nearest_pos.y;
    direction.z = rand_point.z - nearest_pos.z;

    double norm = distance(nearest_pos, rand_point);
    if (norm > step_size) {
      direction.x = (direction.x / norm) * step_size;
      direction.y = (direction.y / norm) * step_size;
      direction.z = (direction.z / norm) * step_size;
    }

    Point3D new_point(
      nearest_pos.x + direction.x,
      nearest_pos.y + direction.y,
      nearest_pos.z + direction.z);

    // Check if path to new point is valid
    bool path_valid = true;
    int num_checks = static_cast<int>(distance(nearest_pos, new_point) / collision_check_resolution_) + 1;
    for (int i = 0; i <= num_checks; ++i) {
      double t = static_cast<double>(i) / num_checks;
      Point3D check_point(
        nearest_pos.x + direction.x * t,
        nearest_pos.y + direction.y * t,
        nearest_pos.z + direction.z * t);

      if (!isStateValid(check_point)) {
        path_valid = false;
        break;
      }
    }

    if (!path_valid) continue;

    // RRT*: Find best parent in neighborhood
    int best_parent = nearest_idx;
    double best_cost = tree[nearest_idx].cost + distance(nearest_pos, new_point);

    if (use_rrt_star_) {
      for (size_t i = 0; i < tree.size(); ++i) {
        double dist = distance(tree[i].position, new_point);
        if (dist < rewire_radius) {
          double new_cost = tree[i].cost + dist;
          if (new_cost < best_cost) {
            // Check if path is collision-free
            bool valid = true;
            int checks = static_cast<int>(dist / collision_check_resolution_) + 1;
            for (int j = 0; j <= checks; ++j) {
              double t = static_cast<double>(j) / checks;
              Point3D check(
                tree[i].position.x + (new_point.x - tree[i].position.x) * t,
                tree[i].position.y + (new_point.y - tree[i].position.y) * t,
                tree[i].position.z + (new_point.z - tree[i].position.z) * t);
              if (!isStateValid(check)) {
                valid = false;
                break;
              }
            }
            if (valid) {
              best_parent = i;
              best_cost = new_cost;
            }
          }
        }
      }
    }

    // Add new node
    int new_idx = tree.size();
    tree.push_back(RRTNode(new_point, best_parent, best_cost));

    // Check if goal reached
    if (distance(new_point, goal) < goal_tolerance) {
      goal_idx = new_idx;
      RCLCPP_INFO(this->get_logger(), "Goal reached after %d iterations", iter);
      break;
    }
  }

  // Extract path
  if (goal_idx == -1) {
    RCLCPP_WARN(this->get_logger(), "No path found to goal");
    return false;
  }

  // Backtrack from goal to start
  path.clear();
  int current_idx = goal_idx;
  while (current_idx != -1) {
    path.push_back(tree[current_idx].position);
    current_idx = tree[current_idx].parent_idx;
  }

  // Reverse to get start-to-goal path
  std::reverse(path.begin(), path.end());

  return true;
}

void PathPlanner::smoothPath(std::vector<Point3D>& path)
{
  if (path.size() < 3) return;

  std::vector<Point3D> smoothed;
  smoothed.push_back(path.front());

  size_t i = 0;
  while (i < path.size() - 1) {
    // Try to skip ahead as far as possible
    size_t j = path.size() - 1;
    bool found = false;

    while (j > i + 1) {
      // Check if direct path from i to j is valid
      bool valid = true;
      double dist = std::sqrt(
        std::pow(path[j].x - path[i].x, 2) +
        std::pow(path[j].y - path[i].y, 2) +
        std::pow(path[j].z - path[i].z, 2));

      int num_checks = static_cast<int>(dist / collision_check_resolution_) + 1;
      for (int k = 0; k <= num_checks; ++k) {
        double t = static_cast<double>(k) / num_checks;
        Point3D check(
          path[i].x + (path[j].x - path[i].x) * t,
          path[i].y + (path[j].y - path[i].y) * t,
          path[i].z + (path[j].z - path[i].z) * t);

        if (!isStateValid(check)) {
          valid = false;
          break;
        }
      }

      if (valid) {
        smoothed.push_back(path[j]);
        i = j;
        found = true;
        break;
      }
      --j;
    }

    if (!found) {
      smoothed.push_back(path[i + 1]);
      i++;
    }
  }

  path = smoothed;
}

void PathPlanner::publishPath(const std::vector<Point3D>& path)
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = map_frame_;
  path_msg.header.stamp = this->now();

  for (const auto& point : path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = point.z;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
  }

  path_pub_->publish(path_msg);
}

void PathPlanner::publishPathVisualization(const std::vector<Point3D>& path)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = map_frame_;
  marker.header.stamp = this->now();
  marker.ns = "planned_path";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.1;  // Line width
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  for (const auto& point : path) {
    geometry_msgs::msg::Point p;
    p.x = point.x;
    p.y = point.y;
    p.z = point.z;
    marker.points.push_back(p);
  }

  path_vis_pub_->publish(marker);
}

}  // namespace path_planner_3d

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<path_planner_3d::PathPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
