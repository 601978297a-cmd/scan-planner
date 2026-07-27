from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_share = get_package_share_directory("m20_scan_bringup")
    planner_yaml = os.path.join(
        bringup_share, "config", "m20_scan_planner.yaml")
    controller_yaml = os.path.join(
        bringup_share, "config", "m20_scan_controller.yaml")
    safety_bridge_yaml = os.path.join(
        bringup_share, "config", "m20_scan_safety_bridge.yaml")
    udp_bridge_yaml = os.path.join(
        bringup_share, "config", "m20_scan_udp_bridge.yaml")
    direct_udp_yaml = os.path.join(
        bringup_share, "config", "m20_scan_direct_udp.yaml")
    control_backend = LaunchConfiguration("control_backend")
    enable_nav_cmd_output = LaunchConfiguration("enable_nav_cmd_output")
    enable_udp_output = LaunchConfiguration("enable_udp_output")
    require_navigation_enable = LaunchConfiguration(
        "require_navigation_enable")
    sensor_pose_adapter_backend = LaunchConfiguration(
        "sensor_pose_adapter_backend")
    sensor_pose_adapter_parameters = [{
        "target_frame": "world",
        "source_frame": "rslidar_front",
        "cloud_topic": "/rslidar_points_front",
        "output_topic": "/scan/sensor_pose",
        "cloud_stamp_topic": "/scan/front_cloud_stamp",
        "synced_cloud_topic": "/scan/front_cloud_synced",
        "max_tf_wait_sec": 0.5,
        "max_latest_tf_age_sec": 0.12,
        "retry_period_sec": 0.02,
        "max_queue_size": 2,
        "use_sim_time": False,
    }]

    return LaunchDescription([
        DeclareLaunchArgument(
            "control_backend",
            default_value="nav_cmd",
            choices=["nav_cmd", "udp", "direct_udp"],
            description="Select exactly one M20 control backend.",
        ),
        DeclareLaunchArgument(
            "sensor_pose_adapter_backend",
            default_value="cpp",
            choices=["cpp", "python"],
            description="Select the concurrent C++ adapter or Python fallback.",
        ),
        DeclareLaunchArgument(
            "enable_nav_cmd_output",
            default_value="false",
            choices=["true", "false"],
            description="Allow the guarded NAV_CMD backend to create the real M20 command publisher.",
        ),
        DeclareLaunchArgument(
            "enable_udp_output",
            default_value="false",
            choices=["true", "false"],
            description="Allow the UDP backend to open the real M20 motion link; it still starts DISARMED.",
        ),
        DeclareLaunchArgument(
            "require_navigation_enable",
            default_value="true",
            choices=["true", "false"],
            description="Require the controller navigation-enable gate.",
        ),
        Node(
            package="m20_sensor_pose_adapter_cpp",
            executable="sensor_pose_adapter_cpp",
            name="sensor_pose_adapter",
            output="screen",
            parameters=sensor_pose_adapter_parameters,
            condition=IfCondition(PythonExpression([
                "'", sensor_pose_adapter_backend, "' == 'cpp'",
            ])),
        ),
        Node(
            package="m20_scan_bringup",
            executable="sensor_pose_adapter",
            name="sensor_pose_adapter",
            output="screen",
            parameters=sensor_pose_adapter_parameters,
            condition=IfCondition(PythonExpression([
                "'", sensor_pose_adapter_backend, "' == 'python'",
            ])),
        ),
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[
                planner_yaml,
                {"fsm.require_navigation_enable": ParameterValue(
                    require_navigation_enable, value_type=bool)},
            ],
            remappings=[
                ("body_pose", "/lio/robo/odom"),
                ("sensor_pose", "/scan/sensor_pose"),
                ("cloud", "/scan/front_cloud_synced"),
                ("move_base_simple/goal", "/move_base_simple/goal"),
                ("initial_path", "/initial_path"),
            ],
        ),
        Node(
            package="scan_planner",
            executable="closed_loop_controller",
            name="closed_loop_controller",
            output="screen",
            parameters=[
                controller_yaml,
                {"require_navigation_enable": ParameterValue(
                    require_navigation_enable, value_type=bool)},
            ],
            remappings=[
                ("planning/bspline", "/planning/bspline"),
                ("body_pose", "/lio/robo/odom"),
                ("cmd_vel", "/scan/cmd_vel_debug"),
            ],
        ),
        Node(
            package="m20_scan_bringup",
            executable="scan_m20_safety_bridge",
            name="scan_m20_safety_bridge",
            output="screen",
            parameters=[
                safety_bridge_yaml,
                {"enable_m20_output": ParameterValue(
                    enable_nav_cmd_output, value_type=bool)},
            ],
            condition=IfCondition(PythonExpression([
                "'", control_backend, "' == 'nav_cmd'",
            ])),
        ),
        Node(
            package="m20_scan_bringup",
            executable="scan_m20_udp_safety_bridge",
            name="scan_m20_udp_safety_bridge",
            output="screen",
            parameters=[
                udp_bridge_yaml,
                {"enable_udp_output": ParameterValue(
                    enable_udp_output, value_type=bool)},
            ],
            condition=IfCondition(PythonExpression([
                "'", control_backend, "' == 'udp'",
            ])),
        ),
        Node(
            package="m20_scan_bringup",
            executable="scan_m20_direct_udp_bridge",
            name="scan_m20_direct_udp_bridge",
            output="screen",
            parameters=[direct_udp_yaml],
            condition=IfCondition(PythonExpression([
                "'", control_backend, "' == 'direct_udp'",
            ])),
        ),
    ])
