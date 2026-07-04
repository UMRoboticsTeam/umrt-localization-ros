"""Launch base ublox_dgnss_node without configuring CFG_* settings."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration


def generate_launch_description():
    device_serial_string = LaunchConfiguration("device_serial_string")
    device_family = LaunchConfiguration("device_family")
    frame_id = LaunchConfiguration("frame_id")

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=TextSubstitution(text="INFO")
    )

    device_family_arg = DeclareLaunchArgument(
        "device_family",
        default_value=TextSubstitution(text="F9P")
    )

    device_serial_string_arg = DeclareLaunchArgument(
        "device_serial_string",
        default_value="GPSF",
        description="Serial string of the base receiver"
    )

    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="gps_left_link",
        description="Frame ID for base receiver messages"
    )

    params_base = [
        {"DEVICE_FAMILY": device_family},
        {"DEVICE_SERIAL_STRING": device_serial_string},
        {"FRAME_ID": frame_id},
    ]

    container_base = ComposableNodeContainer(
        name="ublox_dgnss_moving_base",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        composable_node_descriptions=[
            ComposableNode(
                package="ublox_dgnss_node",
                plugin="ublox_dgnss::UbloxDGNSSNode",
                name="ublox_dgnss",
                namespace="base",
                parameters=params_base
            )
        ]
    )

    container_navsatfix = ComposableNodeContainer(
        name="ublox_nav_sat_fix_hp_base_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        composable_node_descriptions=[
            ComposableNode(
                package="ublox_nav_sat_fix_hp_node",
                plugin="ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode",
                namespace="base",
                name="ublox_nav_sat_fix_hp"
            )
        ]
    )

    return launch.LaunchDescription([
        log_level_arg,
        device_family_arg,
        device_serial_string_arg,
        frame_id_arg,
        container_base,
        container_navsatfix,
    ])