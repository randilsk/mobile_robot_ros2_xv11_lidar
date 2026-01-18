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

### SLAM Toolbox (for mapping)

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

**Keyboard Control:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
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

### Loading a Pre-Saved Map (Localization)

To use a previously saved map for localization:

```bash
ros2 launch slam_toolbox localization_launch.py \
  slam_params_file:=/home/ubuntu/dev_ws/src/my_bot/config/mapper_params_online_async.yaml \
  use_sim_time:=true
```

## Navigation2 (Autonomous Navigation)

Navigation2 enables autonomous path planning and obstacle avoidance.

### Method 1: Using Nav2 Bringup (Recommended)

```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

### Method 2: Manual Launch (Step by Step)

1. **Launch Map Server:**
   ```bash
   ros2 run nav2_map_server map_server \
     --ros-args \
     -p yaml_filename:=my_map_save.yaml \
     -p use_sim_time:=true
   ```

2. **Activate Map Server lifecycle (in a new terminal):**
   ```bash
   ros2 run nav2_util lifecycle_bringup map_server
   ```

3. **Launch AMCL (Adaptive Monte Carlo Localization):**
   ```bash
   ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true
   ```

4. **Activate AMCL lifecycle (in a new terminal):**
   ```bash
   ros2 run nav2_util lifecycle_bringup amcl
   ```

### Setting Navigation Goals

- Use RViz's "2D Goal Pose" tool to set navigation targets
- The robot will autonomously plan and execute paths to reach goals

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
| `/diff_cont/cmd_vel_unstamped` | geometry_msgs/Twist | Velocity commands |
| `/diff_cont/odom` | nav_msgs/Odometry | Odometry data |
| `/joint_states` | sensor_msgs/JointState | Joint states |
| `/tf` | tf2_msgs/TFMessage | Transform tree |

## Troubleshooting

### Lidar rays not visible in Gazebo
- Ensure `<visualize>true</visualize>` is set in `lidar.xacro`

### Robot not moving with teleop
- Check velocity topic remapping
- Verify `use_stamped_vel` parameter matches your command type

### Obstacles rotating in RViz
- Ensure RViz is started with `use_sim_time:=true`
- Verify Fixed Frame is set to `odom`
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
