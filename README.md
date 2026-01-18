# Mobile Robot ROS2 with XV11 Lidar

A ROS 2 differential drive mobile robot simulation with XV11 lidar integration for autonomous navigation and obstacle detection.

## Features

- 🤖 Differential drive mobile robot with accurate odometry
- 📡 XV11 Lidar sensor integration (360-degree laser scanning)
- 🎮 Joystick and keyboard teleoperation support
- 🌍 Gazebo simulation with custom worlds
- 📊 RViz visualization with real-time laser scan data
- 🔧 ROS 2 Control integration (ros2_control)
- 🗺️ SLAM Toolbox integration for mapping and localization
- 🧭 Navigation2 for autonomous navigation and path planning

## System Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo 11
- Python 3.10+

## Dependencies

### Core Dependencies

```bash
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-xacro
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-joy
sudo apt install ros-humble-teleop-twist-joy
sudo apt install ros-humble-teleop-twist-keyboard
```

### SLAM Toolbox (for mapping and localization)

```bash
sudo apt install ros-humble-slam-toolbox
```

### Navigation2 (for autonomous navigation)

```bash
sudo apt update
sudo apt install \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-turtlebot3*
```

### Twist Mux (for velocity command multiplexing)

```bash
sudo apt install ros-humble-twist-mux
```

## Building the Project

```bash
cd ~/dev_Ws
colcon build
source install/setup.bash
```

## Quick Start

### 1. Launch Simulation

**Basic simulation:**
```bash
ros2 launch my_bot launch_sim.launch.py
```

**Simulation with custom world:**
```bash
ros2 launch my_bot launch_sim.launch.py world:=./src/my_bot/worlds/obstacles.world
```

### 2. Start RViz (in a new terminal)

```bash
source ~/dev_Ws/install/setup.bash
ros2 run rviz2 rviz2 -d ~/dev_Ws/src/my_bot/config/main.rviz --ros-args -p use_sim_time:=true
```

### 3. Control the Robot

**Keyboard Control (Direct):**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

**Keyboard Control (Through Twist Mux - Recommended for Nav2):**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_keyboard
```

**Joystick Control:**
- Connect your joystick
- The joystick node is automatically launched with the simulation
- Enable button: Button 6
- Turbo button: Button 7

## SLAM (Mapping)

SLAM Toolbox is used for creating maps of the environment.

### Creating a New Map

1. **Launch the simulation:**
   ```bash
   ros2 launch my_bot launch_sim.launch.py world:=./src/my_bot/worlds/obstacles.world
   ```

2. **Start SLAM Toolbox for mapping:**
   ```bash
   ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/ubuntu/dev_ws/src/my_bot/config/mapper_params_online_async.yaml use_sim_time:=true
   ```

3. **Drive the robot around** using teleop to build the map

4. **Save the map** when finished exploring

## Autonomous Navigation Setup

There are two methods for autonomous navigation: using SLAM Toolbox for localization or using AMCL. Both work with Nav2.

### Method 1: Complete Workflow with SLAM Toolbox (Recommended)

This method uses SLAM Toolbox for localization with a pre-saved map.

**Terminal 1 - Launch Gazebo Simulation:**
```bash
ros2 launch my_bot launch_sim.launch.py world:=./src/my_bot/worlds/obstacles.world
```

**Terminal 2 - Launch RViz:**
```bash
rviz2 -d /home/ubuntu/dev_ws/src/my_bot/config/main.rviz --ros-args -p use_sim_time:=true
```

**Terminal 3 - Launch Twist Mux (for cmd_vel remapping):**
```bash
ros2 run twist_mux twist_mux --ros-args \
  --params-file /home/ubuntu/dev_ws/src/my_bot/config/twist_mux.yaml \
  -r cmd_vel_out:=/diff_cont/cmd_vel_unstamped
```

**Terminal 4 - Launch SLAM Toolbox with Pre-loaded Map:**
```bash
ros2 launch slam_toolbox localization_launch.py \
  slam_params_file:=/home/ubuntu/dev_ws/src/my_bot/config/mapper_params_online_async.yaml \
  use_sim_time:=true
```

**Terminal 5 - Launch Nav2 Stack:**
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

**In RViz:**
1. Add a new Map display
2. Set topic to `/global_costmap/costmap`
3. This shows the loaded map with navigation boundaries clearly
4. Use the **2D Goal Pose** button to set a target position on the map
5. The robot will autonomously navigate to that position

**Optional - Keyboard Control (Terminal 6):**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_keyboard
```
*Note: Keyboard has higher priority than Nav2 in twist_mux, so you can override autonomous navigation*

---

### Method 2: Complete Workflow with AMCL (Alternative)

This method uses AMCL (Adaptive Monte Carlo Localization) for localization.

**Terminal 1 - Launch Gazebo Simulation:**
```bash
ros2 launch my_bot launch_sim.launch.py world:=./src/my_bot/worlds/obstacles.world
```

**Terminal 2 - Launch RViz:**
```bash
rviz2 -d /home/ubuntu/dev_ws/src/my_bot/config/main.rviz --ros-args -p use_sim_time:=true
```

**RViz Configuration:**
- Set **Fixed Frame** to `map`
- In the Map display panel, set **Durability Policy** to `Transient Local`

**Terminal 3 - Launch Twist Mux:**
```bash
ros2 run twist_mux twist_mux --ros-args \
  --params-file /home/ubuntu/dev_ws/src/my_bot/config/twist_mux.yaml \
  -r cmd_vel_out:=/diff_cont/cmd_vel_unstamped
```

**Terminal 4 - Launch AMCL Localization:**
```bash
ros2 launch nav2_bringup localization_launch.py \
  map:=my_map_save.yaml \
  use_sim_time:=true
```

**Terminal 5 - Launch Nav2 Navigation Stack:**
```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  map_subscribe_transient_local:=true
```

**In RViz:**
1. Use **2D Pose Estimate** button to set the robot's initial position on the map
2. Wait for AMCL particle cloud to converge (particles cluster around robot)
3. Add Map display with topic `/global_costmap/costmap` to see navigation boundaries
4. Use **2D Goal Pose** button to set navigation targets
5. The robot will autonomously navigate to the goal

---

## Twist Mux Configuration

Twist Mux multiplexes velocity commands from multiple sources (Nav2, keyboard, joystick) based on priority:

- **Navigation (Nav2)**: Priority 10 (lowest)
- **Keyboard**: Priority 100 (highest)
- **Joystick**: Priority 100 (highest)

Higher priority inputs override lower priority ones. This allows manual override of autonomous navigation.

**Key Topic Mapping:**
- Inputs: `/cmd_vel` (Nav2), `/cmd_vel_keyboard` (keyboard), `/cmd_vel_joy` (joystick)
- Output: `/diff_cont/cmd_vel_unstamped` (robot controller)

---

## Navigation Controls

### Setting Navigation Goals

Use RViz's **2D Goal Pose** tool to set navigation targets. The robot will:
1. Plan a path using the global planner
2. Execute the path while avoiding dynamic obstacles
3. Re-plan if obstacles block the path

### Manual Override

While navigating, you can take manual control using:
- Keyboard teleop (higher priority overrides Nav2)
- Joystick (if configured)

The robot will resume autonomous navigation when manual commands stop.

## Package Structure

```
dev_Ws/
├── src/
│   ├── my_bot/                    # Main robot package
│   │   ├── config/                # Configuration files
│   │   │   ├── my_controllers.yaml
│   │   │   └── main.rviz
│   │   ├── description/           # Robot description (URDF/Xacro)
│   │   │   ├── robot.urdf.xacro
│   │   │   ├── robot_core.xacro
│   │   │   ├── lidar.xacro
│   │   │   └── gazebo_control.xacro
│   │   ├── launch/                # Launch files
│   │   │   ├── launch_sim.launch.py
│   │   │   └── rsp.launch.py
│   │   └── worlds/                # Gazebo worlds
│   │       └── obstacles.world
│   ├── xv11_lidar_python/         # XV11 Lidar driver
│   └── ... (other packages)
└── README.md
```

## Robot Specifications

- **Wheel Radius:** 0.034 m
- **Wheel Separation:** 0.35 m
- **Lidar Range:** 0.3m - 12m
- **Lidar Scan Angle:** 360° (-π to π)
- **Lidar Samples:** 360 points per scan

## Configuration

### Controller Parameters

Edit `src/my_bot/config/my_controllers.yaml` to adjust:
- Update rate
- Wheel parameters
- Velocity limits
- Odometry settings

### Lidar Parameters

Edit `src/my_bot/description/lidar.xacro` to modify:
- Scan range (min/max)
- Update rate
- Sample count
- Visualization settings

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | sensor_msgs/LaserScan | Lidar scan data |
| `/cmd_vel` | geometry_msgs/Twist | Nav2 velocity commands (input to twist_mux) |
| `/cmd_vel_keyboard` | geometry_msgs/Twist | Keyboard velocity commands (input to twist_mux) |
| `/cmd_vel_joy` | geometry_msgs/Twist | Joystick velocity commands (input to twist_mux) |
| `/diff_cont/cmd_vel_unstamped` | geometry_msgs/Twist | Robot controller velocity commands (output from twist_mux) |
| `/diff_cont/odom` | nav_msgs/Odometry | Odometry data |
| `/joint_states` | sensor_msgs/JointState | Joint states |
| `/tf` | tf2_msgs/TFMessage | Transform tree |
| `/map` | nav_msgs/OccupancyGrid | Map for navigation |
| `/global_costmap/costmap` | nav_msgs/OccupancyGrid | Global costmap with inflation |
| `/local_costmap/costmap` | nav_msgs/OccupancyGrid | Local costmap for obstacle avoidance |

## Troubleshooting

### Lidar rays not visible in Gazebo
- Ensure `<visualize>true</visualize>` is set in `lidar.xacro`

### Robot not moving with teleop
- Check velocity topic remapping
- Verify `use_stamped_vel` parameter matches your command type

### Robot not moving with Nav2
- Ensure twist_mux is running with correct output remapping to `/diff_cont/cmd_vel_unstamped`
- Check `ros2 topic info /cmd_vel` shows at least 1 subscription (twist_mux)
- Check `ros2 topic info /diff_cont/cmd_vel_unstamped` shows 1 publisher (twist_mux)
- Verify robot is properly localized (set initial pose with "2D Pose Estimate" in RViz)

### Transform errors (map to odom)
- Ensure localization is running (SLAM Toolbox or AMCL)
- Set initial pose in RViz using "2D Pose Estimate" before navigation
- Check transform tree: `ros2 run tf2_ros tf2_echo map odom`
- Verify `base_frame` in config matches your robot's base frame

### AMCL warnings: "cannot publish pose"
- Use "2D Pose Estimate" in RViz to set initial robot position
- Wait for particle cloud to converge before navigating

### Navigation fails immediately
- Check that map is loaded: `ros2 topic echo /map --once`
- Verify costmaps are visible in RViz
- Ensure goal is on navigable space (not in obstacles or unknown area)

### Obstacles rotating in RViz
- Ensure RViz is started with `use_sim_time:=true`
- Verify Fixed Frame is set correctly (`odom` for teleop, `map` for navigation)
- Check wheel radius matches robot model (0.034m)

### Build errors
```bash
# Clean build
rm -rf build/ install/ log/
colcon build
```

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.


## Author

randilsk

## Acknowledgments

- ROS 2 community
- XV11 Lidar Python driver contributors
- Gazebo simulation platform
