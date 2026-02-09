# Path Planner 3D

3D path planning and control for autonomous underwater exploration using RRT* algorithm.

## Overview

This package provides:
1. **PathPlanner Node** - Plans collision-free 3D paths using RRT* algorithm
2. **PathController Node** - Follows planned paths with 6-DOF velocity control

## Architecture

```
/exploration_goal (from frontier_detection_3d)
        ↓
   PathPlanner Node
   - RRT* path planning
   - Collision checking with octomap
   - Path smoothing
        ↓
/planned_path (nav_msgs/Path)
        ↓
   PathController Node
   - Pure pursuit controller
   - 6-DOF velocity commands
        ↓
/cmd_vel (geometry_msgs/Twist)
```

## Nodes

### 1. PathPlanner Node

**Subscribed Topics:**
- `/exploration_goal` (geometry_msgs/PoseStamped) - Target goal from frontier detector
- `/octomap_binary` (octomap_msgs/Octomap) - 3D occupancy map for collision checking

**Published Topics:**
- `/planned_path` (nav_msgs/Path) - Planned waypoint path
- `/path_visualization` (visualization_msgs/Marker) - Green line showing planned path

**Parameters:** (see [config/planner_params.yaml](config/planner_params.yaml))
- `planning_timeout`: 5.0s - Maximum time for path planning
- `collision_check_resolution`: 0.2m - Resolution for collision checking
- `robot_radius`: 0.5m - Safety radius around robot
- `use_rrt_star`: true - Use RRT* (optimizes path cost)

### 2. PathController Node

**Subscribed Topics:**
- `/planned_path` (nav_msgs/Path) - Path to follow

**Published Topics:**
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands for robot
- `/current_target` (geometry_msgs/PoseStamped) - Current waypoint target

**Parameters:** (see [config/controller_params.yaml](config/controller_params.yaml))
- `control_frequency`: 10Hz - Control loop rate
- `waypoint_tolerance`: 0.5m - Distance to consider waypoint reached
- `max_linear_velocity`: 1.0 m/s - Maximum speed
- `k_linear`: 0.5 - Proportional gain for linear velocity
- `k_angular`: 1.0 - Proportional gain for angular velocity

## Launch Files

### Individual Nodes

**Launch path planner only:**
```bash
ros2 launch path_planner_3d path_planner.launch.py
```

**Launch path controller only:**
```bash
ros2 launch path_planner_3d path_controller.launch.py
```

### Full Navigation System

**Launch both planner and controller:**
```bash
ros2 launch path_planner_3d full_navigation.launch.py
```

## Complete System Launch

To run the entire exploration system:

**Terminal 1: OctoMap Builder**
```bash
cd ~/Documents/frog_lab/exploration/ocean_frontier_exploration
source install/setup.bash
ros2 launch octomap_3d_exploration octomap_builder.launch.py
```

**Terminal 2: Frontier Detector**
```bash
cd ~/Documents/frog_lab/exploration/ocean_frontier_exploration
source install/setup.bash
ros2 launch frontier_detection_3d frontier_detector.launch.py
```

**Terminal 3: Path Planner + Controller**
```bash
cd ~/Documents/frog_lab/exploration/ocean_frontier_exploration
source install/setup.bash
ros2 launch path_planner_3d full_navigation.launch.py
```

**Terminal 4: RViz**
```bash
rviz2
```

## RViz Visualization

Add these displays to visualize the navigation:

1. **Planned Path** (green line)
   - Type: Marker
   - Topic: `/path_visualization`

2. **Current Path** (waypoints)
   - Type: Path
   - Topic: `/planned_path`

3. **Current Target** (purple arrow)
   - Type: PoseStamped
   - Topic: `/current_target`

## Algorithm Details

### RRT* Path Planning

The planner uses **Rapidly-exploring Random Tree Star (RRT*)** algorithm:

1. **Sampling:** Randomly samples 3D points in the planning space
2. **Goal Bias:** 10% probability of sampling the goal to speed up convergence
3. **Rewiring:** RRT* finds optimal parent nodes within a radius to minimize path cost
4. **Collision Checking:** Validates path segments against octomap obstacles
5. **Path Smoothing:** Removes unnecessary waypoints using line-of-sight checks

**Key Features:**
- Plans in full 3D space (not limited to 2D plane)
- Considers robot radius for safe clearance
- Asymptotically optimal (finds better paths with more iterations)
- Timeout protection (5 seconds default)

### Path Following Controller

Simple proportional controller for 6-DOF navigation:

```
v_x = k_linear * (target_x - robot_x)
v_y = k_linear * (target_y - robot_y)
v_z = k_linear * (target_z - robot_z)
ω_z = k_angular * angle_error
```

- Moves to next waypoint when within `waypoint_tolerance`
- Stops when final goal is reached
- Publishes zero velocity when no path is active

## Parameters Tuning

### Planning Performance

- **Increase `planning_timeout`** if paths aren't found in cluttered environments
- **Decrease `collision_check_resolution`** for faster planning (less safe)
- **Increase `robot_radius`** for more conservative obstacle avoidance

### Control Performance

- **Increase `k_linear`** for faster but less stable motion
- **Increase `control_frequency`** for smoother control
- **Decrease `waypoint_tolerance`** for more precise path following

## Dependencies

- ROS2 Jazzy
- rclcpp
- geometry_msgs, nav_msgs, visualization_msgs
- octomap / octomap_msgs
- tf2 / tf2_ros
- Isaac Sim (for robot simulation)

## Expected Behavior

1. **Goal received** → "Planning from (x,y,z) to (x,y,z)"
2. **Path found** → "Path found with N waypoints"
3. **Path smoothed** → "Smoothed to M waypoints" (M < N)
4. **Following path** → Publishes `/cmd_vel` at 10 Hz
5. **Waypoint reached** → "Reached waypoint X/Y"
6. **Goal reached** → "Path completed!" + stops robot

## Troubleshooting

### No path found
- **Cause:** Goal is in collision or unreachable
- **Solution:** Check if goal (yellow sphere) is inside an obstacle in RViz
- **Solution:** Increase `planning_timeout` for harder problems

### Robot not moving
- **Cause:** Isaac Sim not subscribed to `/cmd_vel`
- **Solution:** Verify topic with `ros2 topic echo /cmd_vel`
- **Solution:** Check Isaac Sim is configured to receive velocity commands

### Robot oscillating
- **Cause:** Control gains too high
- **Solution:** Decrease `k_linear` and `k_angular`
- **Solution:** Increase `waypoint_tolerance`

### Path goes through obstacles
- **Cause:** `robot_radius` too small
- **Solution:** Increase `robot_radius` parameter
- **Solution:** Decrease `collision_check_resolution` for more checks

## File Structure

```
path_planner_3d/
├── include/path_planner_3d/
│   ├── path_planner.hpp        # PathPlanner node header
│   └── path_controller.hpp     # PathController node header
├── src/
│   ├── path_planner.cpp        # RRT* planning implementation
│   └── path_controller.cpp     # 6-DOF controller implementation
├── launch/
│   ├── path_planner.launch.py       # Launch planner only
│   ├── path_controller.launch.py    # Launch controller only
│   └── full_navigation.launch.py    # Launch both
├── config/
│   ├── planner_params.yaml     # Planning parameters
│   └── controller_params.yaml  # Control parameters
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Topics Summary

| Topic | Type | Description |
|-------|------|-------------|
| `/exploration_goal` | PoseStamped | Input: Target from frontier detector |
| `/octomap_binary` | Octomap | Input: 3D map for collision checking |
| `/planned_path` | Path | Output: Waypoint path to follow |
| `/path_visualization` | Marker | Output: Green line visualization |
| `/cmd_vel` | Twist | Output: Velocity commands for robot |
| `/current_target` | PoseStamped | Output: Current waypoint being tracked |

## Next Steps

- [ ] Implement dynamic replanning when new obstacles detected
- [ ] Add recovery behaviors (stuck detection, backing up)
- [ ] Implement multiple goal queue management
- [ ] Add path prediction and look-ahead visualization
- [ ] Integrate with multi-robot coordination

---

**Status:** ✅ Built and ready to test
**Author:** Autonomous Exploration Team
**Last Updated:** 2026-02-08
