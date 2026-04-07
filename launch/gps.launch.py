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
    launch_dir = get_package_share_directory('ublox_dgnss')
    launch_path = os.path.join(launch_dir, 'launch','ublox_rover_hpposllh_navsatfix.launch.py')
    '''
    gps_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments={
            'namespace': 'gps_front',
            'DEVICE_FAMILY': 'F9P',
            'DEVICE_SERIAL_STRING': 'GPSF', # Replace with actual serial
            'frame_id': 'gps_front_link',
           # 'device': '/dev/serial/by-path/platform-xhci-hcd.2.auto-usb-0:1.4:1.0',
        }.items()
    )
    '''
    gps_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments={
            'namespace': 'gps_back',
            'DEVICE_FAMILY': 'F9P',
            'DEVICE_SERIAL_STRING': 'GPSB', # Replace with actual serial
            'frame_id': 'gps_back_link',
           # 'device': '/dev/serial/by-path/platform-xhci-hcd.2.auto-usbv2-0:1.4:1.0',
        }.items()
    )
    
    return LaunchDescription([

        #gps_1,
        gps_2,

        
    
    ])

