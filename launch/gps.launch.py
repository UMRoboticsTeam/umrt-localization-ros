"""
Copyright 2026, University of Manitoba Robotics Team
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
Created on Aug 3,2026 by Author: Dev Patel, Senay Yemessghen
"""

"""
UMRT Moving Base RTK GPS launch file

Starts both GPS used for moving-base RTK positioning. Settings come from config/gps_params.yaml.
This file does not start heading_node - see localization.launch.py for that.
"""

"""
Imports
"""
from launch import LaunchDescription, LaunchContext  # CHANGED: added LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import yaml

"""
    Builds the launch description for the two GPS.
    Loads config/gps_params.yaml to get each gps' settings (namespace, serial string, frame ID), then starts the port right away and the starboard 5 seconds later, so that the port is fully up and starts broadcasting corrections.
"""

def generate_launch_description():

    # Location of each gps' individual launch file.
    base_launch_path = PathJoinSubstitution([
        FindPackageShare('umrt-localization-ros'),
        'launch',
        'ublox_mb+r_base.launch.py'
    ])

    rover_launch_path = PathJoinSubstitution([
        FindPackageShare('umrt-localization-ros'),
        'launch',
        'ublox_mb+r_rover.launch.py'
    ])

    # Load the settings for both GPS from one shared config file.
    config_path_substitution = PathJoinSubstitution([
        FindPackageShare('umrt-localization-ros'),
        'config',
        'gps_params.yaml'
    ])
    config_path = config_path_substitution.perform(LaunchContext())

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # Port - broadcasts correction data.
    gps_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_path),
        launch_arguments=config['gps_port'].items()
    )

    # Starboard - receives corrections and publishes RELPOSNED, which heading_node uses to figure out the rover's heading.
    gps_rover = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rover_launch_path),
        launch_arguments=config['gps_starboard'].items()
    )

    # Wait 5 seconds before starting the starboard, so the port is ready and broadcasting corrections first.
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