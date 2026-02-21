# Isaac Sim Integration for Autonomous Exploration

## What Was Added

I've integrated ROS2 `/cmd_vel` subscriber into your OceanSim Isaac Sim extension so the robot will respond to navigation commands.

## Files Modified/Created

### New File:
- `OceanSim/isaacsim/oceansim/utils/cmd_vel_subscriber.py`
  - Creates OmniGraph for ROS2 `/cmd_vel` subscription
  - Applies velocity commands to BlueROV at `/World/rob`

### Modified Files:
- `OceanSim/isaacsim/oceansim/modules/SensorExample_python/scenario.py`
  - Added `CmdVelController` initialization in `setup_scenario()`
  - Added cleanup in `teardown_scenario()`

## How It Works

```
ROS2 /cmd_vel Topic
    ↓
OmniGraph ROS2SubscribeTwist Node
    ↓
IsaacApplyForce Node (velocity mode)
    ↓
BlueROV at /World/rob
```

The system subscribes to `/cmd_vel` (geometry_msgs/Twist) and applies the linear and angular velocities directly to the robot's rigid body.

## Testing the Integration

### 1. Start Isaac Sim
Open Isaac Sim and load your OceanSim extension with the sensor example.

### 2. Load Scenario
- Click "Load" to set up the scene
- Click "Run Scenario" to start simulation
- **Important:** Make sure control mode is NOT "Manual control" (otherwise keyboard will interfere)

### 3. Verify ROS2 Connection
In a terminal, check that Isaac Sim is now subscribed:
```bash
ros2 topic info /cmd_vel --verbose
```

You should see:
```
Subscription count: 1  ← Isaac Sim is now listening!
```

### 4. Test Manual Commands (Optional)
Test the connection by sending a simple command:
```bash
# Move forward
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Stop
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

The robot should move in Isaac Sim!

### 5. Run Full Autonomous System

**Terminal 1: Isaac Sim**
```bash
# Launch Isaac Sim and run your OceanSim scenario
```

**Terminal 2: OctoMap Builder**
```bash
cd ~/Documents/frog_lab/exploration/ocean_frontier_exploration
source install/setup.bash
ros2 launch octomap_3d_exploration octomap_builder.launch.py
```

**Terminal 3: Frontier Detector**
```bash
source install/setup.bash
ros2 launch frontier_detection_3d frontier_detector.launch.py
```

**Terminal 4: Path Planner + Controller**
```bash
source install/setup.bash
ros2 launch path_planner_3d full_navigation.launch.py
```

**Terminal 5: RViz**
```bash
rviz2
```

## Expected Behavior

1. ✅ Isaac Sim subscribes to `/cmd_vel`
2. ✅ Path controller publishes velocity commands
3. ✅ Robot moves in simulation toward frontiers
4. ✅ Autonomous exploration loop runs continuously

## Troubleshooting

### Isaac Sim not subscribing to /cmd_vel
- **Check:** Scenario is loaded and running
- **Check:** No errors in Isaac Sim console
- **Check:** ROS2 bridge extension is enabled

### Robot not moving despite receiving commands
- **Check:** Robot has rigid body physics enabled
- **Check:** Gravity is disabled (underwater mode)
- **Check:** No manual control interfering

### Robot moving erratically
- **Tune:** Controller gains in `path_planner_3d/config/controller_params.yaml`
- **Lower:** `max_linear_velocity` (try 0.5 m/s)
- **Increase:** Damping values in robot setup

### Commands arriving but ignored
- **Check:** OmniGraph node is created (look for `/CmdVelSubscriberGraph` in Isaac Sim)
- **Check:** Force mode is set to "velocity"
- **Check:** Target prim is set to `/World/rob`

## Velocity Command Format

The controller publishes `geometry_msgs/Twist`:

```
linear:
  x: forward/backward velocity (m/s)
  y: left/right velocity (m/s)
  z: up/down velocity (m/s)
angular:
  x: roll rate (rad/s)
  y: pitch rate (rad/s)
  z: yaw rate (rad/s)
```

Currently, the path controller uses:
- `linear.x, linear.y, linear.z` for 3D movement
- `angular.z` for yaw control

## Next Steps

If this works well, you can enhance the integration by:
- [ ] Adding force mode (thrust commands) instead of velocity
- [ ] Implementing thruster allocation for more realistic underwater dynamics
- [ ] Adding safety limits and collision avoidance
- [ ] Integrating DVL feedback for closed-loop velocity control

---

**Quick Test Command:**
```bash
# After loading Isaac Sim scenario:
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

The robot should move forward continuously. Press Ctrl+C to stop.
