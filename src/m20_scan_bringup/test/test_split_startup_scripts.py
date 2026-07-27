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
