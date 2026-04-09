"""
UMRT Robot Launch File

TO-DO:
- Work on QoS Profile for all cameras
- Define Launch Arguments for Cameras to keep a persistent way of getting /dev/video
"""

"""
Imports
"""
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy 
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

"""
Generate Launch Description
"""
def generate_launch_description(): 

    # Path to this package's launch dir
    launch_dir = get_package_share_directory('umrt-localization-ros')
    launch_path = os.path.join(launch_dir, 'launch','ublox.launch.py')
    
    gps_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments={
            'namespace': 'gps_front',
            'device_family': 'F9P',
            'device_serial_string': 'GPSF', # Replace with actual serial
            'frame_id': 'gps_front_link',
        }.items()
    )
    
    gps_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments={
            'namespace': 'gps_back',
            'device_family': 'F9P',
            'device_serial_string': 'GPSB', # Replace with actual serial
            'frame_id': 'gps_back_link',
        }.items()
    )
    
    return LaunchDescription([
        gps_1,
        gps_2,
    ])

