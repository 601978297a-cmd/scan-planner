import re
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_udp_sensor_freshness_timeouts_are_bounded_at_point_eight_seconds():
    config = (
        PACKAGE_ROOT / "config/m20_scan_udp_bridge.yaml"
    ).read_text(encoding="utf-8")

    assert re.search(
        r"^\s+sensor_pose_timeout: 0\.80$", config, re.MULTILINE)
    assert re.search(
        r"^\s+front_cloud_timeout: 0\.80$", config, re.MULTILINE)
