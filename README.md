# Mobile Robot ROS2 with XV11 Lidar

A ROS 2 differential drive mobile robot simulation with XV11 lidar integration for autonomous navigation and obstacle detection.

## Features

- 🤖 Differential drive mobile robot with accurate odometry
- 📡 XV11 Lidar sensor integration (360-degree laser scanning)
- 🎮 Joystick and keyboard teleoperation support
- 🌍 Gazebo simulation with custom worlds
- 📊 RViz visualization with real-time laser scan data
- 🔧 ROS 2 Control integration (ros2_control)

## System Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo 11
- Python 3.10+

## Dependencies

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

## Building the Project

```bash
cd ~/dev_Ws
colcon build
source install/setup.bash
```

## Quick Start

### 1. Launch Simulation

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

## License

[Add your license here]

## Author

Randil SK

## Acknowledgments

- ROS 2 community
- XV11 Lidar Python driver contributors
- Gazebo simulation platform
