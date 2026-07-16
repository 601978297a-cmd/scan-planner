from glob import glob
from setuptools import setup

package_name = "m20_scan_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nvidia",
    maintainer_email="nvidia@example.com",
    description="Dry-run bringup for connecting M20 Lightning-LM localization to SCAN-Planner.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sensor_pose_adapter = m20_scan_bringup.sensor_pose_adapter:main",
            "sensor_pose_from_odom_adapter = m20_scan_bringup.sensor_pose_from_odom_adapter:main",
            "cloud_filter_adapter = m20_scan_bringup.cloud_filter_adapter:main",
        ],
    },
)
