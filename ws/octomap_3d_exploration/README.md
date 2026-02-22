ocean_frontier_exploration/
└── octomap_3d_exploration/          # Your ROS2 package
    ├── CMakeLists.txt               # Build configuration
    ├── package.xml                  # Package metadata & dependencies
    ├── include/                     # C++ header files
    │   └── octomap_3d_exploration/
    │       └── octomap_builder.hpp  # Class declaration
    ├── src/                         # C++ source files
    │   └── octomap_builder.cpp      # Main implementation + main()
    ├── launch/                      # Launch files
    │   └── octomap_builder.launch.py # Python launch script
    └── config/                      # Configuration files
        └── params.yaml              # Runtime parameters


# OctoMap 3D Exploration

A ROS2 package for building persistent 3D occupancy maps using OctoMap for autonomous underwater exploration.

## 📋 Overview

This package integrates stereo camera point clouds into a 3D OctoMap representation, enabling:
- **Persistent 3D mapping** - Accumulates observations over time
- **Probabilistic occupancy fusion** - Bayesian updates for robust mapping
- **Ray casting** - Marks free space along sensor rays
- **Multi-resolution representation** - Efficient octree structure
- **Real-time visualization** - MarkerArray displays for RViz

## 🏗️ System Architecture

```
Stereo Camera → PointCloud2 → OctoMap Builder → 3D Occupancy Map
                                    ↓
                        Visualization (RViz MarkerArray)
                                    ↓
                        Frontier Detection (Future)
```

## 🎯 Features

### Core Functionality
- ✅ **3D Occupancy Mapping** - Full volumetric environment representation
- ✅ **Point Cloud Integration** - Stereo camera point cloud processing
- ✅ **TF Transform Support** - Automatic coordinate frame transformations
- ✅ **Range Filtering** - Configurable min/max sensor range
- ✅ **Probabilistic Updates** - Confidence-based voxel occupancy
- ✅ **Thread-Safe Operations** - Concurrent read/write protection

### Visualization
- ✅ **Occupied Voxels** - Red cubes for obstacles
- ✅ **Free Space** - Green cubes for known free areas
- ✅ **Binary OctoMap** - Compact map representation
- ✅ **Configurable Decimation** - Adjustable visualization density

## 📦 Dependencies

### ROS2 Packages
- `rclcpp` - ROS2 C++ client library
- `sensor_msgs` - PointCloud2 messages
- `geometry_msgs` - Pose and transform messages
- `visualization_msgs` - MarkerArray for RViz
- `tf2` / `tf2_ros` - Transform system
- `pcl_ros` / `pcl_conversions` - Point Cloud Library integration
- `octomap` - 3D occupancy mapping library
- `octomap_msgs` - OctoMap ROS messages

### System Libraries
- `liboctomap-dev` - OctoMap core library
- `libflann-dev` - Fast nearest neighbor search
- PCL (Point Cloud Library)

### Installation
```bash
# Install dependencies
sudo apt update
sudo apt install ros-jazzy-octomap ros-jazzy-octomap-msgs ros-jazzy-octomap-ros \
                 liboctomap-dev libflann-dev ros-jazzy-pcl-ros ros-jazzy-pcl-conversions
```

## 🚀 Building

```bash
# Navigate to workspace
cd ~/Documents/frog_lab/exploration/ocean_frontier_exploration

# Build the package
colcon build --packages-select octomap_3d_exploration --symlink-install

# Source the workspace
source install/setup.bash
```

## ⚙️ Configuration

### Parameters (`config/params.yaml`)

```yaml
octomap_builder:
  ros__parameters:
    # Map Parameters
    resolution: 0.15              # Voxel size (meters)
    max_depth: 16                 # Octree depth (determines max map size)
    
    # Sensor Range
    max_range: 20.0               # Maximum trusted sensor range (meters)
    min_range: 0.3                # Minimum sensor range (meters)
    
    # Coordinate Frames
    frame_id: "UW_camera_world"   # Map/world frame
    robot_frame: "base_link"      # Robot base frame
    
    # Update Rates
    publish_rate: 3.0             # Map publishing frequency (Hz)
    
    # Probabilistic Mapping (Simulator-optimized)
    prob_hit: 0.9                 # Hit probability (high confidence)
    prob_miss: 0.2                # Miss probability (high confidence)
    occupancy_threshold: 0.5      # Occupied/free decision boundary
    clamping_threshold_min: 0.01  # Minimum probability clamp
    clamping_threshold_max: 0.99  # Maximum probability clamp
    
    # Visualization
    visualize_free_space: true    # Show free voxels
    visualize_unknown_space: false # Show unknown voxels
    visualization_decimation: 5   # Show every Nth free voxel
```

### Key Parameter Explanations

| Parameter | Effect | Recommended |
|-----------|--------|-------------|
| `resolution` | Voxel size - smaller = more detail | 0.10 - 0.20 m |
| `max_depth` | Maximum map extent: `resolution × 2^max_depth` | 16 (±9.8km) |
| `max_range` | Sensor trust distance | 15-30 m (simulator) |
| `prob_hit` | Confidence in obstacle detection | 0.7-0.9 |
| `prob_miss` | Confidence in free space | 0.2-0.4 |

**Map Extent Calculation:**
```
Max extent = resolution × 2^max_depth
Example: 0.15 × 2^16 = 9,830 meters (±4.9km from origin)
```

## 🎮 Usage

### Launch the OctoMap Builder

```bash
ros2 launch octomap_3d_exploration octomap_builder.launch.py
```

### Subscribe to Input Topic

The node subscribes to:
- **Topic:** `/UW_Camera_Stereo_pointcloud`
- **Type:** `sensor_msgs/PointCloud2`
- **Frame:** Any (automatically transformed to map frame)

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/octomap_binary` | `octomap_msgs/Octomap` | Binary octree (compact) |
| `/octomap_full` | `octomap_msgs/Octomap` | Full octree with probabilities |
| `/occupied_cells_vis` | `visualization_msgs/MarkerArray` | Red cubes (obstacles) |
| `/free_cells_vis` | `visualization_msgs/MarkerArray` | Green cubes (free space) |
| `/unknown_cells_vis` | `visualization_msgs/MarkerArray` | Blue cubes (frontiers - future) |

## 📊 Visualization in RViz

### Setup RViz

1. **Set Fixed Frame:** `UW_camera_world` (or your map frame)

2. **Add Displays:**
   - Grid (reference)
   - MarkerArray → `/occupied_cells_vis` (obstacles)
   - MarkerArray → `/free_cells_vis` (free space)
   - PointCloud2 → `/UW_Camera_Stereo_pointcloud` (sensor input)

3. **Optional:**
   - Add TF display to see coordinate frames

### Expected Visualization

```
🟥 Red cubes    = Occupied voxels (obstacles)
🟢 Green cubes  = Free voxels (known empty space)
⚪ White points = Live point cloud from camera
```

## 🧠 How It Works

### 1. Point Cloud Processing Pipeline

```
Input: PointCloud2 from stereo camera
  ↓
Transform to map frame (using TF)
  ↓
Filter invalid points (NaN, Inf)
  ↓
Filter by range (min_range < distance < max_range)
  ↓
Insert into OctoMap with ray casting
  ↓
Update voxel probabilities
  ↓
Publish map and visualizations
```

### 2. Ray Casting

For each point in the point cloud:
1. **Cast ray** from sensor origin to point
2. **Mark FREE** all voxels along the ray (probability decreases)
3. **Mark OCCUPIED** the endpoint voxel (probability increases)

```
Sensor ◉ ─────→ ─────→ ─────→ ─────→ ■ Obstacle
        FREE   FREE   FREE   FREE   OCCUPIED
```

### 3. Probabilistic Fusion

Each observation updates voxel probability using Bayesian inference:

```cpp
// Occupied voxel (ray endpoint)
log_odds += log(prob_hit / (1 - prob_hit))

// Free voxel (along ray)
log_odds -= log(prob_miss / (1 - prob_miss))

// Clamp to prevent overconfidence
log_odds = clamp(log_odds, min, max)
```

### 4. Persistent Memory

- Map persists for entire node lifetime
- Accumulates ALL observations
- Self-corrects with new data
- Handles dynamic environments

## 🔧 Troubleshooting

### No Map Building

**Check point cloud is publishing:**
```bash
ros2 topic hz /UW_Camera_Stereo_pointcloud
```

**Check TF frames:**
```bash
ros2 run tf2_ros tf2_echo UW_camera_world stereo_camera_link
```

### Empty Visualization

**Verify topics publishing:**
```bash
ros2 topic list | grep octomap
ros2 topic echo /octomap_binary --once
```

**Check RViz Fixed Frame:**
- Must match `frame_id` parameter (`UW_camera_world`)

### Map Not Growing

**Check filtering:**
- Verify `max_range` is appropriate for your sensor
- Ensure `min_range` doesn't filter out all points

**Monitor with logging:**
```bash
# Look for "Map growth" messages in node output
```

## 📈 Performance

### Typical Metrics (Simulator)

| Metric | Value |
|--------|-------|
| Input rate | 10-30 Hz |
| Processing time | 10-30 ms per cloud |
| Output rate | 2-3 Hz |
| Memory growth | ~50 MB per 500k voxels |
| CPU usage | 10-30% (single core) |

### Memory Usage

```
Startup:     ~100 KB (empty octree)
After 10s:   ~5 MB (50k voxels)
After 60s:   ~50 MB (500k voxels)
After 300s:  ~150 MB (1.5M voxels)
```

**Note:** Memory grows with OBSERVED area, not `max_depth`!

## 🎯 Tuning for Different Scenarios

### Real-World Underwater Robot
```yaml
prob_hit: 0.7                # Conservative (sensor noise)
prob_miss: 0.4
max_range: 10.0              # Limited by water clarity
clamping_threshold_min: 0.12
clamping_threshold_max: 0.97
```

### Simulator (Current Config)
```yaml
prob_hit: 0.9                # Aggressive (perfect sensor)
prob_miss: 0.2
max_range: 20.0              # Extended range
clamping_threshold_min: 0.01
clamping_threshold_max: 0.99
```

### Dynamic Environments
```yaml
prob_hit: 0.65               # Less certain
prob_miss: 0.45
clamping_threshold_min: 0.15 # Tighter clamps
clamping_threshold_max: 0.85 # Allow faster changes
```

## 🔮 Future Work

- [ ] Frontier detection (finding unexplored areas)
- [ ] Frontier clustering (grouping exploration targets)
- [ ] Best frontier selection (information gain + cost)
- [ ] Exploration goal publishing
- [ ] Integration with navigation stack
- [ ] Map saving/loading from disk
- [ ] Multi-robot collaborative mapping

## 📚 References

- [OctoMap Library](https://octomap.github.io/)
- [OctoMap Paper](http://www2.informatik.uni-freiburg.de/~hornung/pub/hornung13auro.pdf)
- [ROS2 Documentation](https://docs.ros.org/)

## 📄 License

MIT

## 👤 Author

Developed for underwater autonomous exploration research.

## 🙏 Acknowledgments

- OctoMap library by Hornung et al.
- ROS2 community
- Point Cloud Library (PCL)