import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'my_bot'

    # Include robot launch
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'launch_robot.launch.py')
        ])
    )

    # Include XV11 lidar launch
    xv11_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('xv11_lidar_python'), 'launch', 'xv11_lidar.launch.py')
        ]),
        launch_arguments={
            'port': '/dev/ttyUSB0',  # Adjust to your lidar's serial port
            'frame_id': 'laser_frame'  # Match the URDF frame name
        }.items()
    )

    return LaunchDescription([
        robot_launch,
        xv11_launch,
    ])
