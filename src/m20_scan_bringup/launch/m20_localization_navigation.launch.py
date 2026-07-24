import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    super_lio_share = get_package_share_directory("super_lio")
    bringup_share = get_package_share_directory("m20_scan_bringup")

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(super_lio_share, "launch", "relocation_3d_bbs.py")
        ),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_share,
                "launch",
                "m20_scan_dry_run.launch.py",
            )
        ),
        launch_arguments={
            "control_backend": "direct_udp",
            "enable_nav_cmd_output": "false",
            "enable_udp_output": "false",
            "require_navigation_enable": "false",
        }.items(),
    )

    return LaunchDescription([
        localization,
        navigation,
    ])
