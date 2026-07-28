from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_ROOT = PACKAGE_ROOT.parents[1] / "scripts"


def _read(name):
    return (SCRIPTS_ROOT / name).read_text(encoding="utf-8")


def test_localization_script_only_starts_super_lio_relocation():
    script = _read("start_m20_localization.sh")

    assert "ros2 launch super_lio relocation_points.py" in script
    assert "m20_scan_dry_run.launch.py" not in script


def test_navigation_script_preserves_combined_launch_arguments():
    script = _read("start_m20_navigation.sh")

    assert "ros2 launch m20_scan_bringup" in script
    assert "m20_scan_dry_run.launch.py" in script
    assert "control_backend:=direct_udp" in script
    assert "enable_nav_cmd_output:=false" in script
    assert "enable_udp_output:=false" in script
    assert "require_navigation_enable:=false" in script
    assert "relocation_points.py" not in script


def test_localization_rviz_script_launches_both_processes_directly():
    script = _read("localization_rviz.sh")

    assert "ros2 launch super_lio relocation_points.py &" in script
    assert 'rviz2 -d "$RVIZ_CONFIG" &' in script
    assert "start_m20_localization.sh" not in script
    assert "start_m20_rviz.sh" not in script
    assert "wait -n" in script
    assert "trap cleanup EXIT" in script
    assert "kill \"$pid\"" in script


def test_mode3_script_launches_navigation_directly():
    script = _read("navimode3.sh")

    assert "ros2 launch m20_scan_bringup" in script
    assert "m20_mode3_navigation.launch.py" in script
    assert "start_m20_mode3_navigation.sh" not in script
