from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_share = get_package_share_directory("m20_scan_bringup")
    planner_yaml = os.path.join(bringup_share, "config", "m20_scan_planner.yaml")
    controller_yaml = os.path.join(bringup_share, "config", "m20_scan_controller.yaml")

    return LaunchDescription([
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
                "cloud_topic": "/rslidar_points",
                "body_pose_topic": "/lightning/odom",
                "base_frame": "base_link",
                "output_topic": "/scan/sensor_pose",
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
                ("cloud", "/rslidar_points"),
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
    ])
