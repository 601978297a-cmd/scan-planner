from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2

import rclpy

from m20_scan_bringup import sensor_pose_adapter as adapter_module
from m20_scan_bringup.sensor_pose_adapter import SensorPoseAdapter


class CapturingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class OrderedCapturingPublisher(CapturingPublisher):
    def __init__(self, name, events):
        super().__init__()
        self.name = name
        self.events = events

    def publish(self, message):
        self.events.append(self.name)
        super().publish(message)


class FixedTransformBuffer:
    def __init__(self, transform):
        self.transform = transform

    def lookup_transform(self, *_args, **_kwargs):
        return self.transform


class DelayedTransformBuffer:
    def __init__(self, transform):
        self.transform = transform
        self.available = False

    def lookup_transform(self, *_args, **_kwargs):
        if not self.available:
            raise RuntimeError("Extrapolation into the future")
        return self.transform


class NoopTransformListener:
    spin_thread = None

    def __init__(self, *_args, **kwargs):
        type(self).spin_thread = kwargs.get("spin_thread")


def _make_node(monkeypatch):
    monkeypatch.setattr(
        "m20_scan_bringup.sensor_pose_adapter.tf2_ros.TransformListener",
        NoopTransformListener,
    )
    rclpy.init()
    node = SensorPoseAdapter()
    assert NoopTransformListener.spin_thread is False
    return node


def _make_cloud():
    cloud = PointCloud2()
    cloud.header.stamp.sec = 123
    cloud.header.stamp.nanosec = 456
    cloud.header.frame_id = "rslidar_front"
    return cloud


def _make_transform():
    transform = TransformStamped()
    transform.header.frame_id = "map"
    transform.child_frame_id = "rslidar_front"
    transform.transform.translation.x = 0.32
    transform.transform.rotation.w = 1.0
    return transform


def test_cloud_is_published_with_matching_pose_after_tf_arrives(monkeypatch):
    node = _make_node(monkeypatch)
    pose_capture = CapturingPublisher()
    stamp_capture = CapturingPublisher()
    cloud_capture = CapturingPublisher()
    node.pose_pub = pose_capture
    node.cloud_stamp_pub = stamp_capture
    node.synced_cloud_pub = cloud_capture
    try:
        cloud = _make_cloud()
        node.tf_buffer = FixedTransformBuffer(_make_transform())
        node.cloud_callback(cloud)
        assert not pose_capture.messages
        assert not stamp_capture.messages
        assert not cloud_capture.messages

        node.process_pending_clouds()

        assert len(stamp_capture.messages) == 1
        assert stamp_capture.messages[0].stamp == cloud.header.stamp
        assert stamp_capture.messages[0].frame_id == "rslidar_front"
        assert cloud_capture.messages == [cloud]

        assert len(pose_capture.messages) == 1
        pose = pose_capture.messages[0]
        assert pose.header.stamp == cloud.header.stamp
        assert pose.header.frame_id == "map"
        assert pose.child_frame_id == "rslidar_front"
        assert pose.pose.pose.position.x == 0.32
        assert pose.pose.pose.orientation.w == 1.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_synced_pair_publishes_stamp_before_pose_and_cloud(monkeypatch):
    node = _make_node(monkeypatch)
    events = []
    node.pose_pub = OrderedCapturingPublisher("pose", events)
    node.cloud_stamp_pub = OrderedCapturingPublisher("stamp", events)
    node.synced_cloud_pub = OrderedCapturingPublisher("cloud", events)
    try:
        node._publish_synced_pair(_make_cloud(), _make_transform())

        assert events == ["stamp", "pose", "cloud"]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cloud_waits_nonblocking_until_exact_tf_is_available(monkeypatch):
    node = _make_node(monkeypatch)
    pose_capture = CapturingPublisher()
    cloud_capture = CapturingPublisher()
    node.pose_pub = pose_capture
    node.synced_cloud_pub = cloud_capture
    try:
        buffer = DelayedTransformBuffer(_make_transform())
        node.tf_buffer = buffer
        cloud = _make_cloud()
        node.cloud_callback(cloud)

        node.process_pending_clouds()
        assert len(node.pending_clouds) == 1
        assert not pose_capture.messages
        assert not cloud_capture.messages

        buffer.available = True
        node.process_pending_clouds()
        assert not node.pending_clouds
        assert pose_capture.messages[0].header.stamp == cloud.header.stamp
        assert cloud_capture.messages == [cloud]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cloud_is_dropped_after_tf_wait_timeout(monkeypatch):
    node = _make_node(monkeypatch)
    pose_capture = CapturingPublisher()
    cloud_capture = CapturingPublisher()
    node.pose_pub = pose_capture
    node.synced_cloud_pub = cloud_capture
    try:
        node.tf_buffer = DelayedTransformBuffer(_make_transform())
        node.cloud_callback(_make_cloud())
        node.pending_clouds[0].received_at -= node.max_tf_wait_sec + 0.1

        node.process_pending_clouds()

        assert not node.pending_clouds
        assert not pose_capture.messages
        assert not cloud_capture.messages
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_main_uses_two_thread_executor(monkeypatch):
    events = []

    class FakeNode:
        def destroy_node(self):
            events.append("destroy_node")

    class FakeExecutor:
        def __init__(self, *, num_threads):
            events.append(("executor", num_threads))

        def add_node(self, node):
            events.append(("add_node", node))

        def spin(self):
            events.append("executor_spin")

        def shutdown(self):
            events.append("executor_shutdown")

    node = FakeNode()
    monkeypatch.setattr(
        adapter_module, "MultiThreadedExecutor", FakeExecutor, raising=False)
    monkeypatch.setattr(adapter_module, "SensorPoseAdapter", lambda: node)
    monkeypatch.setattr(
        adapter_module.rclpy, "init", lambda args=None: events.append(("init", args)))
    monkeypatch.setattr(
        adapter_module.rclpy, "spin", lambda _node: events.append("rclpy_spin"))
    monkeypatch.setattr(
        adapter_module.rclpy, "shutdown", lambda: events.append("shutdown"))

    adapter_module.main(args=["--test"])

    assert events == [
        ("init", ["--test"]),
        ("executor", 2),
        ("add_node", node),
        "executor_spin",
        "executor_shutdown",
        "destroy_node",
        "shutdown",
    ]
