"""
Launch file for XV-11 Lidar driver.

This launch file starts the xv11_lidar_python node with configurable parameters.
You can use this file directly or copy it into your own project's launch directory.

Usage:
    ros2 launch xv11_lidar_python xv11_lidar.launch.py
    
    With custom parameters:
    ros2 launch xv11_lidar_python xv11_lidar.launch.py port:=/dev/ttyAMA0 frame_id:=xiaomi_lidar
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for XV-11 Lidar node."""

    # Declare launch arguments with defaults
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the lidar device'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_frame',
        description='TF frame ID for the laser scan'
    )
    
    range_min_arg = DeclareLaunchArgument(
        'range_min',
        default_value='0.06',
        description='Minimum range in meters'
    )
    
    range_max_arg = DeclareLaunchArgument(
        'range_max',
        default_value='13.0',
        description='Maximum range in meters'
    )

    # Create the xv11_lidar node
    xv11_node = Node(
        package='xv11_lidar_python',
        executable='xv11_lidar',
        name='xv11_lidar',
        parameters=[
            {'port': LaunchConfiguration('port')},
            {'frame_id': LaunchConfiguration('frame_id')},
            {'range_min': LaunchConfiguration('range_min')},
            {'range_max': LaunchConfiguration('range_max')},
        ],
        output='screen',  # Show node output in terminal
    )

    # Static transform publisher: laser_frame -> frame_id (xv11_lidar)
    # This creates the TF link that RViz needs to display lidar data
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_frame_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'laser_frame', LaunchConfiguration('frame_id')],
        output='screen',
    )

    return LaunchDescription([
        port_arg,
        frame_id_arg,
        range_min_arg,
        range_max_arg,
        xv11_node,
        static_tf_node,
    ])
