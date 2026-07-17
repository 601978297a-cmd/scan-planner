from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_share = get_package_share_directory("m20_scan_bringup")
    planner_yaml = os.path.join(bringup_share, "config", "m20_scan_planner.yaml")
    controller_yaml = os.path.join(bringup_share, "config", "m20_scan_controller.yaml")
    safety_bridge_yaml = os.path.join(bringup_share, "config", "m20_scan_safety_bridge.yaml")
    udp_bridge_yaml = os.path.join(
        bringup_share, "config", "m20_scan_udp_bridge.yaml")
    control_backend = LaunchConfiguration("control_backend")

    return LaunchDescription([
        DeclareLaunchArgument(
            "control_backend",
            default_value="nav_cmd",
            choices=["nav_cmd", "udp"],
            description="Select exactly one guarded M20 control backend.",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_world_for_scan_markers",
            output="screen",
            arguments=["0", "0", "0", "0", "0", "0", "map", "world"],
        ),
        Node(
            package="m20_scan_bringup",
            executable="sensor_pose_from_odom_adapter",
            name="sensor_pose_adapter",
            output="screen",
            parameters=[{
                "target_frame": "map",
                "source_frame": "rslidar_front",
                "cloud_topic": "/rslidar_points_front",
                "body_pose_topic": "/lightning/odom",
                "base_frame": "base_link",
                "output_topic": "/scan/sensor_pose",
                "cloud_stamp_topic": "/scan/front_cloud_stamp",
                "lookup_timeout_sec": 1.0,
                "max_body_pose_delta_sec": 0.5,
                "use_sim_time": False,
            }],
        ),
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[planner_yaml],
            remappings=[
                ("body_pose", "/lightning/odom"),
                ("sensor_pose", "/scan/sensor_pose"),
                ("cloud", "/rslidar_points_front"),
                ("move_base_simple/goal", "/move_base_simple/goal"),
                ("initial_path", "/initial_path"),
            ],
        ),
        Node(
            package="scan_planner",
            executable="closed_loop_controller",
            name="closed_loop_controller",
            output="screen",
            parameters=[controller_yaml],
            remappings=[
                ("planning/bspline", "/planning/bspline"),
                ("body_pose", "/lightning/odom"),
                ("cmd_vel", "/scan/cmd_vel_debug"),
            ],
        ),
        Node(
            package="m20_scan_bringup",
            executable="scan_m20_safety_bridge",
            name="scan_m20_safety_bridge",
            output="screen",
            parameters=[safety_bridge_yaml],
            condition=IfCondition(PythonExpression([
                "'", control_backend, "' == 'nav_cmd'",
            ])),
        ),
        Node(
            package="m20_scan_bringup",
            executable="scan_m20_udp_safety_bridge",
            name="scan_m20_udp_safety_bridge",
            output="screen",
            parameters=[udp_bridge_yaml],
            condition=IfCondition(PythonExpression([
                "'", control_backend, "' == 'udp'",
            ])),
        ),
    ])
