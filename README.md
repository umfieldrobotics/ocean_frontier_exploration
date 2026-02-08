# ocean_frontier_exploration
ROS 2–based frontier exploration framework for underwater robots in OceanSim (Isaac Sim). The repository provides mapping, frontier detection, and control pipelines that interface with Isaac Sim via ROS 2 topics, enabling autonomous exploration using acoustic and visual sensing in simulated underwater environments.

Added extension of stereo camera in Ocean sim

todo: complete the readme

todo: mk=aking occupancy grid for sonar and stereo 

step 1: making an occupancy grid with stereo under water

I need: 1- stereo/left/image_raw, /stereo/right/image_raw

/stereo/left/camera_info
/stereo/right/camera_info

stereo camera iumages basically

2. we need the calibration parameters as well

3. /stereo/depth/image_raw   (sensor_msgs/Image, float32 meters)

4. /stereo/points   (sensor_msgs/PointCloud2)



Our Plan on High Level 

Isaac Sim (sensors + physics)
   ↓ ROS2 Bridge
Depth / PointCloud2
   ↓
ROS mapping node
   ↓
nav_msgs/OccupancyGrid
   ↓
RViz visualization

STEP 0 — Verify OceanSim + Isaac Sim Baseline
0.1 Clone OceanSim

(If already cloned, skim this section)

git clone https://github.com/umfieldrobotics/OceanSim.git

0.2 Launch an OceanSim Scene (Isaac Sim side)

Launch Isaac Sim using the provided launcher

❌ Do not use system Python

❌ Do not launch via pip

Load an OceanSim world:

ROV / underwater scene

Verify inside Isaac Sim only:

Robot spawns correctly

Stereo cameras appear in the Stage Tree

Camera outputs are visible in the Isaac viewport

✅ If cameras render correctly in Isaac, proceed
❌ If not, stop — ROS will not fix broken sensors

STEP 1 — Enable ROS 2 Bridge (Isaac Sim)
1.1 Enable Required Extensions

In Isaac Sim:

Window → Extensions


Enable:

omni.isaac.ros2_bridge

omni.isaac.sensor

omni.isaac.core

🔁 Restart Isaac Sim after enabling

1.2 Set ROS 2 Distribution

In Isaac Sim:

Isaac Utils → ROS2 Bridge → Settings


Set:

ROS_DISTRO = jazzy


Confirm Isaac terminal prints:

Using ROS 2 Jazzy

STEP 2 — Publish Stereo Camera Topics (CRITICAL)

OceanSim may include cameras, but they are NOT ROS-visible until connected via OmniGraph.

2.1 Identify Camera Prims

In the Stage Tree, locate exact camera paths (examples):

/World/Robot/stereo_left
/World/Robot/stereo_right


📌 Write down the exact paths — names may differ.

2.2 Create ROS Camera Graphs (Stereo)

In Isaac Sim:

Isaac Utils → ROS2 → Camera


Create two camera graphs:

Left Camera

Camera Prim: /World/Robot/stereo_left

Topics:

/stereo/left/image_raw → sensor_msgs/Image

/stereo/left/camera_info → sensor_msgs/CameraInfo

Frame ID:

stereo_left_optical

Right Camera

Camera Prim: /World/Robot/stereo_right

Topics:

/stereo/right/image_raw

/stereo/right/camera_info

Frame ID:

stereo_right_optical


📌 Use ROS2 Camera Info Helper
📌 Frame IDs must be unique

2.3 Verify From ROS Side

Open a new terminal:

ros2 topic list


You must see:

/stereo/left/image_raw
/stereo/right/image_raw
/stereo/left/camera_info
/stereo/right/camera_info


❌ If missing → stop and fix before proceeding

STEP 3 — Generate Depth / Point Cloud (ROS Side)

Two options are available.
✅ Option A is strongly recommended.

Option A (RECOMMENDED): Stereo → PointCloud2
3.1 Install Stereo Processing Pipeline
sudo apt install ros-jazzy-stereo-image-proc

3.2 Run Stereo Processing Node
ros2 run stereo_image_proc stereo_image_proc


Subscribes to:

/stereo/left/image_raw
/stereo/right/image_raw


Publishes:

/stereo/points2   (sensor_msgs/PointCloud2)


Verify:

ros2 topic echo /stereo/points2

Option B: Isaac-Published Depth (Optional)

If OceanSim already publishes depth:

/stereo/depth/image_raw


You may skip stereo processing, but:

❌ Less realistic

❌ Less calibration-faithful

STEP 4 — Publish TF and Odometry (NON-NEGOTIABLE)
4.1 Required TF Tree

Your system must contain:

map
 └── odom
     └── base_link
         ├── stereo_left_optical
         └── stereo_right_optical


OceanSim typically publishes:

/tf

/odom

4.2 Verify TF Tree
ros2 run tf2_tools view_frames


Open:

frames.pdf


❌ If camera frames are missing → mapping will fail

STEP 5 — Convert PointCloud → Occupancy Grid

This step generates the actual 2D map.

5.1 Install Mapping Node
git clone https://github.com/jkk-research/pointcloud_to_grid.git
cd pointcloud_to_grid
colcon build


Source workspace:

source install/setup.bash

5.2 Run Mapping Node
ros2 run pointcloud_to_grid pointcloud_to_grid_node \
  --ros-args \
  -p cloud_topic:=/stereo/points2 \
  -p map_frame:=map \
  -p resolution:=0.1 \
  -p height_min:=-2.0 \
  -p height_max:=0.5


Publishes:

/map   (nav_msgs/OccupancyGrid)

STEP 6 — RViz Visualization (Recommended)
6.1 Launch RViz
rviz2

6.2 RViz Configuration

Set:

Fixed Frame = map


Add displays:

Map → /map

PointCloud2 → /stereo/points2

TF

✅ If map appears → Pipeline complete
❌ If not → TF or timestamps are wrong (99% of failures)

STEP 7 — Isaac-Side Visualization (Optional)

Inside Isaac Sim you may visualize:

Point clouds

Rays

Voxel debug views

Useful for:

Sensor validation

Noise modeling

Underwater distortion analysis

⚠️ Isaac Sim does not publish nav_msgs/OccupancyGrid by default.

Common Failure Checklist (Bookmark This)

❌ Camera frame missing from TF

❌ Incorrect optical frame orientation

❌ Missing odom → map transform

❌ Timestamp mismatch (Isaac paused or desynced)