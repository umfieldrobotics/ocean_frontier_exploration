# Ocean Frontier Exploration

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

Autonomous 3D frontier-based exploration system for underwater robotics using OctoMap, A* path planning, and 6-DOF control.

---

## 🌊 Overview

**Ocean Frontier Exploration** is a complete autonomous exploration pipeline for underwater vehicles. The system builds real-time 3D occupancy maps, identifies exploration frontiers at the boundaries of known and unknown space, plans collision-free paths, and autonomously navigates to explore uncharted environments.

### Key Features

- 🗺️ **Real-time 3D Mapping** - Probabilistic occupancy grid mapping using OctoMap
- 🎯 **Frontier Detection** - Identifies boundaries between explored and unexplored space  
- 🛤️ **3D Path Planning** - A* algorithm for collision-free path generation
- 🎮 **6-DOF Control** - Full 3D velocity control for underwater navigation
- 📊 **Multi-resolution Processing** - Efficient octree-based representation
- 🎨 **RViz Visualization** - Real-time 3D visualization of maps, frontiers, and paths
- 🤖 **Fully Autonomous** - End-to-end exploration without human intervention

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Stereo Camera (Simulation)                    │
└────────────────────────┬────────────────────────────────────────┘
                         ↓
            /UW_Camera_Stereo_pointcloud
                         ↓
┌─────────────────────────────────────────────────────────────────┐
│              Package 1: octomap_3d_exploration                   │
│  • Ray casting from sensor origin                               │
│  • Probabilistic Bayesian fusion                                │
│  • Persistent 3D occupancy map                                  │
│  • FREE/OCCUPIED voxel classification                           │
└────────────────────────┬────────────────────────────────────────┘
                         ↓
              /octomap_binary
                         ↓
┌─────────────────────────────────────────────────────────────────┐
│             Package 2: frontier_detection_3d                     │
│  • Find FREE voxels adjacent to UNKNOWN space                   │
│  • Cluster nearby frontiers                                     │
│  • Calculate information gain                                   │
│  • Score by: gain × exp(-λ × distance)                          │
│  • Select best exploration target                               │
└────────────────────────┬────────────────────────────────────────┘
                         ↓
              /exploration_goal
                         ↓
┌─────────────────────────────────────────────────────────────────┐
│              Package 3: path_planner_3d                          │
│  • A* path planning in 3D (26-connected grid search)            │
│  • Collision checking with octomap                              │
│  • Path smoothing                                               │
│  • Pure pursuit controller                                      │
│  • 6-DOF velocity commands                                      │
└────────────────────────┬────────────────────────────────────────┘
                         ↓
                   /cmd_vel
                         ↓
┌─────────────────────────────────────────────────────────────────┐
│                  Underwater Robot (Isaac Sim)                    │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📦 Packages

### 1. **octomap_3d_exploration**

Real-time 3D occupancy mapping from stereo camera point clouds.

#### Features
- Probabilistic ray casting and sensor fusion
- Persistent map accumulation
- Multi-resolution octree storage (±9.8km map extent)
- RViz MarkerArray visualization

#### Key Topics
| Topic | Type | Description |
|-------|------|-------------|
| **Subscribe:** `/UW_Camera_Stereo_pointcloud` | `PointCloud2` | Stereo camera input |
| **Publish:** `/octomap_binary` | `Octomap` | Binary octree |
| **Publish:** `/occupied_cells_vis` | `MarkerArray` | 🟥 Red cubes (obstacles) |
| **Publish:** `/free_cells_vis` | `MarkerArray` | 🟩 Green cubes (free space) |

**📖 [Full Documentation](octomap_3d_exploration/README.md)**

---

### 2. **frontier_detection_3d**

3D frontier-based exploration using OctoMap.

#### Features
- Detects FREE voxels adjacent to UNKNOWN space
- Frontier clustering
- Information gain calculation
- Distance-weighted goal selection

#### Key Topics
| Topic | Type | Description |
|-------|------|-------------|
| **Subscribe:** `/octomap_binary` | `Octomap` | 3D map |
| **Publish:** `/exploration_goal` | `PoseStamped` | Exploration target |
| **Publish:** `/frontier_markers` | `MarkerArray` | 🔵 Blue cubes (frontiers) |
| **Publish:** `/best_frontier_marker` | `MarkerArray` | 🟡 Yellow sphere (selected) |

**📖 [Full Documentation](frontier_detection_3d/README.md)**

---

### 3. **path_planner_3d**

3D path planning and control using A* algorithm.

#### Features
- A* grid-based path planning with collision checking
- 26-connected 3D neighborhood for smooth paths
- Guaranteed optimal paths on discretized grid
- Path smoothing
- 6-DOF velocity control
- Pure pursuit controller

#### Key Topics
| Topic | Type | Description |
|-------|------|-------------|
| **Subscribe:** `/exploration_goal` | `PoseStamped` | Target goal |
| **Subscribe:** `/octomap_binary` | `Octomap` | Map for collision checking |
| **Publish:** `/planned_path` | `Path` | Waypoint sequence |
| **Publish:** `/path_visualization` | `Marker` | 🟢 Green path line |
| **Publish:** `/cmd_vel` | `Twist` | Velocity commands |

**📖 [Full Documentation](path_planner_3d/README.md)**

---

## 🚀 Quick Start

### Prerequisites

- **ROS2 Jazzy** ([Installation Guide](https://docs.ros.org/en/jazzy/Installation.html))
- **Ubuntu 24.04**
- **Isaac Sim** (for underwater robot simulation)

### Dependencies

```bash
# Install system dependencies
sudo apt update
sudo apt install -y \
  ros-jazzy-octomap \
  ros-jazzy-octomap-msgs \
  ros-jazzy-octomap-ros \
  liboctomap-dev \
  libflann-dev \
  ros-jazzy-pcl-ros \
  ros-jazzy-pcl-conversions
```

### Building

```bash
# Clone repository
cd ~/your_workspace
git clone https://github.com/umfieldrobotics/ocean_frontier_exploration.git
cd ocean_frontier_exploration

# Build all packages
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### Running the System

#### **Terminal 1: OctoMap Builder**
```bash
source install/setup.bash
ros2 launch octomap_3d_exploration octomap_builder.launch.py
```

#### **Terminal 2: Frontier Detector**
```bash
source install/setup.bash
ros2 launch frontier_detection_3d frontier_detector.launch.py
```

#### **Terminal 3: Path Planner + Controller**
```bash
source install/setup.bash
ros2 launch path_planner_3d full_navigation.launch.py
```

#### **Terminal 4: Visualization**
```bash
rviz2
```

**In RViz, set Fixed Frame to `UW_camera_world` and add:**
- MarkerArray → `/occupied_cells_vis` (🟥 obstacles)
- MarkerArray → `/free_cells_vis` (🟩 free space)
- MarkerArray → `/frontier_markers` (🔵 frontiers)
- MarkerArray → `/best_frontier_marker` (🟡 target)
- Marker → `/path_visualization` (🟢 path)

---

## 📊 System Performance

| Component | Update Rate | CPU Usage | Memory |
|-----------|-------------|-----------|--------|
| OctoMap Builder | 0.4 Hz | 15-30% | ~50 MB |
| Frontier Detector | 1 Hz | 5-10% | ~10 MB |
| Path Planner | On-demand | 20-40% | ~20 MB |
| Path Controller | 10 Hz | <5% | ~5 MB |

---

## 🎨 Visualization Guide

### RViz Display Setup

| Display | Topic | Color | Description |
|---------|-------|-------|-------------|
| MarkerArray | `/occupied_cells_vis` | 🟥 Red | Obstacles |
| MarkerArray | `/free_cells_vis` | 🟩 Green | Free space |
| MarkerArray | `/frontier_markers` | 🔵 Blue | Frontiers |
| MarkerArray | `/best_frontier_marker` | 🟡 Yellow | Selected target |
| Marker | `/path_visualization` | 🟢 Green | Planned path |
| PoseStamped | `/current_target` | 🟣 Purple | Waypoint |

---

## 🧮 Algorithms

### Probabilistic Mapping
Ray casting with Bayesian updates:
```cpp
log_odds += log(prob_hit / (1 - prob_hit))      // On hit
log_odds -= log(prob_miss / (1 - prob_miss))    // On miss
```

### Frontier Detection
```python
for each FREE voxel:
    if has_unknown_neighbor:
        → frontier
        
clusters = cluster_by_distance(frontiers)
score = gain × exp(-λ × distance)
```

### A* Path Planning
Grid-based search with 26-connected neighborhood:
```python
# Discretize 3D space into voxel grid
grid_resolution = 0.3m

# Expand all 26 neighbors (face, edge, vertex)
for neighbor in 26_connected_neighbors:
    f_cost = g_cost + euclidean_distance(neighbor, goal)

# Guaranteed optimal path on discretized grid
# Typical planning time: 0.1-2 seconds
```

### Path Following
Proportional controller:
```python
v = k_linear × (target - robot)
ω = k_angular × angle_error
```

---

## 🐛 Troubleshooting

**No map building?**
- Check: `ros2 topic hz /UW_Camera_Stereo_pointcloud`
- Verify TF: `ros2 run tf2_ros tf2_echo UW_camera_world base_link`

**No frontiers detected?**
- Wait 20-30s for map to build
- Reduce `min_frontier_size` parameter

**No path found?**
- Check goal not in obstacle
- Increase `planning_timeout`

**Robot not moving?**
- Verify: `ros2 topic echo /cmd_vel`
- Check Isaac Sim subscribed to `/cmd_vel`

---

## 🗂️ Repository Structure

```
ocean_frontier_exploration/
├── octomap_3d_exploration/          # 3D Mapping
├── frontier_detection_3d/           # Exploration
├── path_planner_3d/                 # Planning & Control
├── docs/                            # Documentation
└── README.md                        # This file
```

---

## 🔮 Future Work

- [ ] Dynamic replanning
- [ ] Multi-robot exploration
- [ ] Map persistence
- [ ] Real-world deployment

---

## 📚 References

- **OctoMap:** Hornung et al., "OctoMap: An Efficient Probabilistic 3D Mapping Framework" (2013)
- **A* Algorithm:** Hart, Nilsson, and Raphael, "A Formal Basis for the Heuristic Determination of Minimum Cost Paths" (1968)
- **Frontier Exploration:** Yamauchi, "A Frontier-Based Approach for Autonomous Exploration" (1997)

---

## 📄 License

MIT License - see [LICENSE](LICENSE)

---

## 👥 Authors

**University of Michigan Field Robotics Lab**

For questions: umfieldrobotics@umich.edu

---

**🌊 Happy Exploring! 🤖**
