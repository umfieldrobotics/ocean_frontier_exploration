# Frontier Detection 3D

ROS2 package for 3D frontier-based autonomous exploration using OctoMap.

## Overview

Detects exploration frontiers (boundaries between FREE and UNKNOWN space) in 3D occupancy maps and publishes optimal exploration goals.

## Features

- ✅ 3D frontier detection from OctoMap
- ✅ Multi-resolution voxel processing
- ✅ Frontier clustering
- ✅ Information gain calculation
- ✅ Distance-weighted goal selection
- ✅ Real-time goal publishing

## Dependencies

- `rclcpp` - ROS2 C++ client
- `octomap` / `octomap_msgs` - 3D mapping
- `geometry_msgs` - Goal messages
- `visualization_msgs` - RViz markers
- `tf2` / `tf2_ros` - Transform system

## Installation
```bash
cd ~/your_workspace
colcon build --packages-select frontier_detection_3d --symlink-install
source install/setup.bash
```

## Usage

### Launch Frontier Detector

**Prerequisites:** `octomap_3d_exploration` must be running!
```bash
ros2 launch frontier_detection_3d frontier_detector.launch.py
```

## Topics

### Subscribes
- `/octomap_binary` - Binary octree from mapping

### Publishes
- `/exploration_goal` - Next exploration target (PoseStamped)
- `/frontier_markers` - All frontier voxels (blue cubes)
- `/best_frontier_marker` - Selected target (yellow sphere)

## Parameters
```yaml
frontier_detector:
  ros__parameters:
    detection_rate: 1.0           # Hz
    min_frontier_size: 5          # Minimum voxels per cluster
    cluster_radius: 1.0           # Clustering distance (m)
    information_radius: 2.0       # Information gain calculation radius (m)
    lambda_distance: 0.1          # Distance cost weight
    map_frame: "UW_camera_world"
    robot_frame: "base_link"
    refinement_depth: 14          # Octree depth for detection
```

## Algorithm

1. **Frontier Detection** - Find FREE voxels with UNKNOWN neighbors
2. **Clustering** - Group nearby frontiers
3. **Scoring** - `score = gain × exp(-λ × distance)`
4. **Selection** - Choose highest-scoring cluster
5. **Publishing** - Send goal to navigation

## Visualization

Add to RViz:
- MarkerArray → `/frontier_markers` (blue)
- MarkerArray → `/best_frontier_marker` (yellow)
- PoseStamped → `/exploration_goal` (arrow)

## License

MIT