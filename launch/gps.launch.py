"""
UMRT Moving Base RTK GPS launch file
"""

"""
Imports
"""
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    launch_dir = get_package_share_directory('umrt-localization-ros')

    base_launch_path = os.path.join(
        launch_dir,
        'launch',
        'ublox_mb+r_base.launch.py'
    )
    rover_launch_path = os.path.join(
        launch_dir,
        'launch',
        'ublox_mb+r_rover.launch.py'
    )

    gps_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_path),
        launch_arguments={
            'namespace': 'gps_left',
            'device_family': 'F9P',
            'device_serial_string': 'GPSF',  # left/base receiver serial
            'frame_id': 'gps_left_link',
        }.items()
    )

    gps_rover = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rover_launch_path),
        launch_arguments={
            'namespace': 'gps_right',
            'device_family': 'F9P',
            'device_serial_string': 'GPSB',  # right/rover receiver serial
            'frame_id': 'gps_right_link',
        }.items()
    )

    gps_rover_delayed = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='Moving base started. Waiting 5s before starting rover...'),
            gps_rover
        ]
    )

    return LaunchDescription([
        gps_base,
        gps_rover_delayed,
    ])