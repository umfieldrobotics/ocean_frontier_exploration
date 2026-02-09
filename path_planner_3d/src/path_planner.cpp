#include "path_planner_3d/path_planner.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace path_planner_3d
{

// Grid index structure for 3D A*
struct GridIndex
{
  int x, y, z;

  GridIndex(int x_ = 0, int y_ = 0, int z_ = 0) : x(x_), y(y_), z(z_) {}

  bool operator==(const GridIndex& other) const {
    return x == other.x && y == other.y && z == other.z;
  }

  // Hash function for unordered_map
  struct Hash {
    std::size_t operator()(const GridIndex& idx) const {
      return std::hash<int>()(idx.x) ^
             (std::hash<int>()(idx.y) << 1) ^
             (std::hash<int>()(idx.z) << 2);
    }
  };
};

// A* Node structure
struct AStarNode
{
  GridIndex index;
  Point3D position;
  double g_cost;        // Cost from start
  double h_cost;        // Heuristic cost to goal
  double f_cost;        // Total cost (g + h)
  GridIndex parent;

  AStarNode(const GridIndex& idx, const Point3D& pos, double g, double h, const GridIndex& p)
    : index(idx), position(pos), g_cost(g), h_cost(h), f_cost(g + h), parent(p) {}

  // For priority queue (min-heap based on f_cost)
  bool operator>(const AStarNode& other) const {
    return f_cost > other.f_cost;
  }
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
  this->declare_parameter("grid_resolution", 0.3);
  this->declare_parameter("robot_radius", 0.5);
  this->declare_parameter("max_planning_range", 50.0);
  this->declare_parameter("heuristic_weight", 1.0);
  this->declare_parameter("max_expansions", 60000);

  // Get parameters
  map_frame_ = this->get_parameter("map_frame").as_string();
  robot_frame_ = this->get_parameter("robot_frame").as_string();
  planning_timeout_ = this->get_parameter("planning_timeout").as_double();
  grid_resolution_ = this->get_parameter("grid_resolution").as_double();
  robot_radius_ = this->get_parameter("robot_radius").as_double();
  max_planning_range_ = this->get_parameter("max_planning_range").as_double();
  heuristic_weight_ = this->get_parameter("heuristic_weight").as_double();
  max_expansions_ = this->get_parameter("max_expansions").as_int();

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

  RCLCPP_INFO(this->get_logger(), "PathPlanner initialized with A* algorithm");
  RCLCPP_INFO(this->get_logger(), "  Map frame: %s", map_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Robot frame: %s", robot_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Planning timeout: %.1f s", planning_timeout_);
  RCLCPP_INFO(this->get_logger(), "  Grid resolution: %.2f m", grid_resolution_);
  RCLCPP_INFO(this->get_logger(), "  Robot radius: %.2f m", robot_radius_);
  RCLCPP_INFO(this->get_logger(), "  Heuristic weight: %.1f", heuristic_weight_);
  RCLCPP_INFO(this->get_logger(), "  Max expansions: %d", max_expansions_);
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

  auto start_time = this->now();
  RCLCPP_INFO(this->get_logger(), "Starting A* planning...");

  // Helper functions
  auto worldToGrid = [this](const Point3D& point) -> GridIndex {
    return GridIndex(
      static_cast<int>(std::round(point.x / grid_resolution_)),
      static_cast<int>(std::round(point.y / grid_resolution_)),
      static_cast<int>(std::round(point.z / grid_resolution_)));
  };

  auto gridToWorld = [this](const GridIndex& idx) -> Point3D {
    return Point3D(
      idx.x * grid_resolution_,
      idx.y * grid_resolution_,
      idx.z * grid_resolution_);
  };

  auto euclideanDistance = [](const Point3D& a, const Point3D& b) -> double {
    return std::sqrt(
      std::pow(a.x - b.x, 2) +
      std::pow(a.y - b.y, 2) +
      std::pow(a.z - b.z, 2));
  };

  // Convert start and goal to grid coordinates
  GridIndex start_idx = worldToGrid(start);
  GridIndex goal_idx = worldToGrid(goal);
  Point3D goal_pos = gridToWorld(goal_idx);

  // Priority queue for open list (min-heap)
  std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;

  // Maps to track costs and parents
  std::unordered_map<GridIndex, double, GridIndex::Hash> g_costs;
  std::unordered_map<GridIndex, GridIndex, GridIndex::Hash> parents;
  std::unordered_set<GridIndex, GridIndex::Hash> closed_list;

  // Initialize start node
  double h_start = heuristic_weight_ * euclideanDistance(start, goal_pos);
  open_list.push(AStarNode(start_idx, start, 0.0, h_start, start_idx));
  g_costs[start_idx] = 0.0;
  parents[start_idx] = start_idx;

  // 26-connected 3D neighborhood
  const std::vector<std::array<int, 3>> neighbors = {
    // 6-connected (face neighbors)
    {{1, 0, 0}}, {{-1, 0, 0}}, {{0, 1, 0}}, {{0, -1, 0}}, {{0, 0, 1}}, {{0, 0, -1}},
    // 12 edge neighbors
    {{1, 1, 0}}, {{1, -1, 0}}, {{-1, 1, 0}}, {{-1, -1, 0}},
    {{1, 0, 1}}, {{1, 0, -1}}, {{-1, 0, 1}}, {{-1, 0, -1}},
    {{0, 1, 1}}, {{0, 1, -1}}, {{0, -1, 1}}, {{0, -1, -1}},
    // 8 vertex neighbors
    {{1, 1, 1}}, {{1, 1, -1}}, {{1, -1, 1}}, {{1, -1, -1}},
    {{-1, 1, 1}}, {{-1, 1, -1}}, {{-1, -1, 1}}, {{-1, -1, -1}}
  };

  int expansions = 0;
  bool goal_found = false;

  while (!open_list.empty() && expansions < max_expansions_) {
    // Check timeout
    if ((this->now() - start_time).seconds() > planning_timeout_) {
      RCLCPP_WARN(this->get_logger(), "Planning timeout after %d expansions", expansions);
      break;
    }

    // Get node with lowest f-cost
    AStarNode current = open_list.top();
    open_list.pop();

    // Skip if already processed
    if (closed_list.count(current.index)) {
      continue;
    }

    // Mark as visited
    closed_list.insert(current.index);
    expansions++;

    // Check if goal reached
    if (current.index == goal_idx) {
      goal_found = true;
      RCLCPP_INFO(this->get_logger(), "Goal reached after %d expansions", expansions);
      break;
    }

    // Expand neighbors
    for (const auto& offset : neighbors) {
      GridIndex neighbor_idx(
        current.index.x + offset[0],
        current.index.y + offset[1],
        current.index.z + offset[2]);

      // Skip if already in closed list
      if (closed_list.count(neighbor_idx)) {
        continue;
      }

      // Convert to world coordinates
      Point3D neighbor_pos = gridToWorld(neighbor_idx);

      // Check if neighbor is valid (collision-free)
      if (!isStateValid(neighbor_pos)) {
        continue;
      }

      // Calculate movement cost
      double move_cost = euclideanDistance(current.position, neighbor_pos);
      double tentative_g = current.g_cost + move_cost;

      // Check if this path to neighbor is better
      if (g_costs.count(neighbor_idx) && tentative_g >= g_costs[neighbor_idx]) {
        continue;
      }

      // Update cost and parent
      g_costs[neighbor_idx] = tentative_g;
      parents[neighbor_idx] = current.index;

      // Calculate heuristic
      double h_cost = heuristic_weight_ * euclideanDistance(neighbor_pos, goal_pos);

      // Add to open list
      open_list.push(AStarNode(neighbor_idx, neighbor_pos, tentative_g, h_cost, current.index));
    }
  }

  if (!goal_found) {
    RCLCPP_WARN(this->get_logger(), "No path found to goal after %d expansions", expansions);
    return false;
  }

  // Reconstruct path by backtracking from goal to start
  path.clear();
  GridIndex current_idx = goal_idx;

  while (!(current_idx == start_idx)) {
    path.push_back(gridToWorld(current_idx));

    if (!parents.count(current_idx)) {
      RCLCPP_ERROR(this->get_logger(), "Path reconstruction failed - broken parent chain");
      return false;
    }

    current_idx = parents[current_idx];
  }

  path.push_back(start);  // Add start position

  // Reverse to get start-to-goal order
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

      int num_checks = static_cast<int>(dist / grid_resolution_) + 1;
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
