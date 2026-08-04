"""
Copyright 2026, University of Manitoba Robotics Team
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
Created on Aug 3,2026 by Author: Dev Patel, Senay Yemessghen
"""

"""
UMRT Localization launch file.

Launches everything needed for heading estimation together:
  - both GPS receivers (via gps.launch.py)
  - the heading_node

Run this instead of gps.launch.py when you want the full localization
"""
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    launch_dir = get_package_share_directory('umrt-localization-ros')

    gps_launch_path = os.path.join(
        launch_dir,
        'launch',
        'gps.launch.py'
    )

    # Both GPS, exactly as started by gps.launch.py.
    gps = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gps_launch_path)
    )

    # heading_node, listening to the starboard's RELPOSNED topic.
    heading_node = Node(
        package='umrt-localization-ros',
        executable='heading_node',
        name='heading_node',
        parameters=[{'relpos_topic': '/gps_starboard/ubx_nav_rel_pos_ned'}]
    )

    return LaunchDescription([
        gps,
        heading_node,
    ])