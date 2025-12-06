# Using xv11_lidar_python Launch Files

This guide explains how to create and use launch files for the `xv11_lidar_python` driver in your own ROS 2 project.

## Quick Start: Use the included launch files

Once you have built the `xv11_lidar_python` package, you can launch the driver directly:

### Using Python launch file
```bash
ros2 launch xv11_lidar_python xv11_lidar.launch.py
```

### Using XML launch file
```bash
ros2 launch xv11_lidar_python xv11_lidar.launch.xml
```

### With custom parameters
```bash
# Override the serial port
ros2 launch xv11_lidar_python xv11_lidar.launch.py port:=/dev/ttyAMA0

# Override multiple parameters
ros2 launch xv11_lidar_python xv11_lidar.launch.py \
    port:=/dev/ttyAMA0 \
    frame_id:=xiaomi_lidar \
    range_max:=12.0
```

## Creating a launch file in your own project

If you want to integrate the xv11_lidar_python driver into your own ROS 2 project, follow these steps:

### 1. Create a launch directory in your project
```bash
mkdir -p ~/colcon_ws/src/your_project/launch
```

### 2. Create a Python launch file

Copy this into `launch/my_robot.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch description for my robot with xv11_lidar."""

    # Arguments for xv11_lidar
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the lidar'
    )

    # Lidar node
    xv11_node = Node(
        package='xv11_lidar_python',
        executable='xv11_lidar',
        name='xv11_lidar',
        parameters=[
            {'port': LaunchConfiguration('lidar_port')},
            {'frame_id': 'lidar_link'},
            {'range_min': 0.06},
            {'range_max': 13.0},
        ],
        output='screen',
    )

    # Your other nodes here...
    # my_other_node = Node(...)

    return LaunchDescription([
        lidar_port_arg,
        xv11_node,
        # Add other nodes to the list
    ])
```

### 3. Create an XML launch file (alternative)

Copy this into `launch/my_robot.launch.xml`:

```xml
<?xml version="1.0"?>
<launch>
  <!-- Arguments -->
  <arg name="lidar_port" default="/dev/ttyUSB0" description="Serial port for the lidar"/>

  <!-- XV-11 Lidar node -->
  <node pkg="xv11_lidar_python" exec="xv11_lidar" name="xv11_lidar" output="screen">
    <param name="port" value="$(arg lidar_port)"/>
    <param name="frame_id" value="lidar_link"/>
    <param name="range_min" value="0.06"/>
    <param name="range_max" value="13.0"/>
  </node>

  <!-- Your other nodes here... -->

</launch>
```

### 4. Include launch files in your setup.py

Update your project's `setup.py` to include launch files:

```python
from setuptools import setup
from glob import glob
import os

setup(
    name='your_project',
    version='0.0.1',
    packages=['your_project'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/your_project']),
        ('share/your_project', ['package.xml']),
        (os.path.join('share', 'your_project', 'launch'),
            glob(os.path.join('launch', '*.launch.[pxy]*'))),
    ],
    # ... rest of setup.py
)
```

### 5. Run your launch file

```bash
colcon build
source install/setup.bash
ros2 launch your_project my_robot.launch.py
```

## Available Parameters

The xv11_lidar_python node accepts the following parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port` | string | `/dev/ttyXV11` | Serial port device path |
| `frame_id` | string | `xv11_lidar` | TF frame ID for the laser scan |
| `range_min` | float | `0.06` | Minimum valid range in meters |
| `range_max` | float | `13.0` | Maximum valid range in meters |

## Output

The node publishes:
- **Topic**: `/scan` (type: `sensor_msgs/msg/LaserScan`)
- **Frame ID**: Configurable via the `frame_id` parameter

## Example: Multi-node launch file

Here's a more complete example that launches the lidar along with other nodes:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Complete robot launch with lidar, mapping, and navigation."""

    # Get paths
    xv11_pkg_dir = get_package_share_directory('xv11_lidar_python')
    
    # Arguments
    lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for lidar'
    )

    # Launch xv11_lidar using its launch file
    xv11_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(xv11_pkg_dir, 'launch', 'xv11_lidar.launch.py')
        ),
        launch_arguments={
            'port': LaunchConfiguration('lidar_port'),
            'frame_id': 'lidar_link',
        }.items(),
    )

    # Your other nodes...
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar_link'],
    )

    return LaunchDescription([
        lidar_port,
        xv11_launch,
        tf_node,
    ])
```

This approach uses `IncludeLaunchDescription` to compose launch files, which is a clean way to integrate the xv11_lidar driver into larger robot systems.

## Troubleshooting

### Launch file not found
- Make sure your `setup.py` includes the launch directory in `data_files`
- Rebuild: `colcon build --packages-select your_project`
- Source: `source install/setup.bash`

### Node doesn't start
- Check the serial port: `ls -la /dev/ttyUSB*`
- Verify permissions: `sudo usermod -a -G dialout $USER` (then logout/login)
- Check logs: `ros2 topic echo /scan` to see if data is being published

### Wrong frame_id in visualizations
- Ensure the frame_id parameter matches your TF tree
- Set up a TF broadcaster: `ros2 run tf2_ros static_transform_publisher ...`
