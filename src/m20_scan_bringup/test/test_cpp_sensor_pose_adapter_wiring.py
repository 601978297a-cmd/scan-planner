from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE_ROOT.parent


def _read(path):
    return path.read_text(encoding="utf-8")


def test_cpp_adapter_is_default_with_python_fallback():
    launch = _read(PACKAGE_ROOT / "launch/m20_scan_dry_run.launch.py")

    assert '"sensor_pose_adapter_backend"' in launch
    assert 'default_value="cpp"' in launch
    assert 'choices=["cpp", "python"]' in launch
    assert 'package="m20_sensor_pose_adapter_cpp"' in launch
    assert 'executable="sensor_pose_adapter_cpp"' in launch
    assert 'package="m20_scan_bringup"' in launch
    assert 'executable="sensor_pose_adapter"' in launch


def test_cpp_adapter_preserves_topics_and_three_execution_paths():
    source = _read(
        WORKSPACE_SRC /
        "m20_sensor_pose_adapter_cpp/src/sensor_pose_adapter.cpp")

    assert '"/rslidar_points_front"' in source
    assert '"/scan/sensor_pose"' in source
    assert '"/scan/front_cloud_stamp"' in source
    assert '"/scan/front_cloud_synced"' in source
    assert "tf_buffer_, this, true" in source
    assert "cloud_options.callback_group = cloud_callback_group_" in source
    assert "match_callback_group_" in source
    assert "MultiThreadedExecutor" in source
    assert "ExecutorOptions(), 2" in source
