from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]


def _read(path):
    return path.read_text(encoding="utf-8")


def test_combined_launch_starts_relocation_without_rviz_and_real_udp_scan():
    launch = _read(
        PACKAGE_ROOT / "launch/m20_localization_navigation.launch.py")

    assert 'get_package_share_directory("super_lio")' in launch
    assert '"relocation.py"' in launch
    assert '"rviz": "false"' in launch
    assert '"m20_scan_dry_run.launch.py"' in launch
    assert '"control_backend": "udp"' in launch
    assert '"enable_udp_output": "true"' in launch


def test_combined_startup_script_uses_only_version1_workspaces():
    script = _read(
        REPOSITORY_ROOT / "scripts/start_m20_localization_navigation.sh")

    assert (
        "/home/nvidia/scanplanner版本1/Super-LIO/install/setup.bash"
        in script
    )
    assert (
        "/home/nvidia/scanplanner版本1/SCAN-Planner/install/setup.bash"
        in script
    )
    assert "/home/nvidia/Super-LIO/install/setup.bash" not in script
    assert "m20_localization_navigation.launch.py" in script


def test_rviz_script_starts_only_scan_rviz():
    script = _read(REPOSITORY_ROOT / "scripts/start_m20_rviz.sh")

    assert (
        "/home/nvidia/scanplanner版本1/Super-LIO/install/setup.bash"
        in script
    )
    assert (
        "/home/nvidia/scanplanner版本1/SCAN-Planner/install/setup.bash"
        in script
    )
    assert "m20_scan.rviz" in script
    assert "ros2 launch" not in script
