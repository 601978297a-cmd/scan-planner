import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("m20_scan_bringup")
    smac_yaml = os.path.join(
        bringup_share, "config", "m20_smac_global.yaml")
    default_map_yaml = (
        "/home/nvidia/Super-LIO/src/super_lio/map/map.yaml")
    map_yaml = LaunchConfiguration("map")

    scan_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_share, "launch", "m20_scan_dry_run.launch.py")),
        launch_arguments={
            "navi_mode": "3",
            "control_backend": "direct_udp",
            "enable_nav_cmd_output": "false",
            "enable_udp_output": "false",
            "require_navigation_enable": "false",
            "use_static_map_collision": "true",
            "double_cylinder_radius": "0.35",
            "static_map_inflation_radius": "0.35",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value=default_map_yaml,
            description="Absolute path to the Nav2 map YAML.",
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                smac_yaml,
                {
                    "yaml_filename": map_yaml,
                    "frame_id": "world",
                },
            ],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[smac_yaml],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="mode3_lifecycle_manager",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["map_server", "planner_server"],
            }],
        ),
        Node(
            package="m20_scan_bringup",
            executable="smac_to_scan_bridge",
            name="smac_to_scan_bridge",
            output="screen",
            parameters=[{
                "global_frame": "world",
                "goal_topic": "/move_base_simple/goal",
                "path_topic": "/initial_path",
                "planner_action": "/compute_path_to_pose",
                "planner_id": "GridBased",
            }],
        ),
        scan_navigation,
    ])
