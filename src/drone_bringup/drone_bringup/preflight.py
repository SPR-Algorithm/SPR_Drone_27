"""Bounded sensor observations and fail-closed checks."""
from collections import deque
import math
import time


def stream_metrics(samples, now_ns):
    if len(samples) < 2:
        raise ValueError("fewer than two samples")
    stamps = [s["stamp_ns"] for s in samples]
    gaps = [(b - a) / 1e9 for a, b in zip(stamps, stamps[1:])]
    if any(g <= 0 for g in gaps):
        raise ValueError("timestamp did not strictly increase")
    if stamps[0] <= 0 or (now_ns - stamps[-1]) / 1e9 < -0.01:
        raise ValueError("timestamp zero or future host time")
    return {"count": len(stamps), "rate_hz": (len(stamps) - 1) * 1e9 / (stamps[-1] - stamps[0]),
            "max_gap_s": max(gaps), "age_s": (now_ns - stamps[-1]) / 1e9}


def validate_streams(samples, rig, now_ns, include_camera=True, include_imu=True,
                     allan_sampling=False):
    result = {}
    topics = {}
    if include_camera:
        for key in ("left_image_topic", "right_image_topic",
                    "left_info_topic", "right_info_topic"):
            topics[rig["camera"][key]] = rig["camera"]
    if include_imu:
        topics[rig["imu"]["topic"]] = rig["imu"]
    width, height, _ = map(int, rig["camera"]["profile"].split("x"))
    for topic, cfg in topics.items():
        observations = samples.get(topic, [])
        metrics = stream_metrics(observations, now_ns)
        if metrics["rate_hz"] < cfg["min_rate_hz"]:
            raise ValueError(f"{topic}: rate below configured minimum: {metrics}")
        if max(metrics["max_gap_s"], metrics["age_s"]) > cfg["max_gap_s"]:
            raise ValueError(f"{topic}: stale or gapped stream: {metrics}")
        if cfg is rig["camera"]:
            if any((s["width"], s["height"]) != (width, height) for s in observations):
                raise ValueError(f"{topic}: profile mismatch")
            if any(not s["valid"] for s in observations):
                raise ValueError(f"{topic}: invalid image payload or camera_info")
        else:
            if any(s["frame_id"] != cfg["frame_id"] or not s["valid"] for s in observations):
                raise ValueError(f"{topic}: invalid IMU frame or numeric payload")
            if allan_sampling:
                from spr_calibration.allan import require_allan_sampling
                metrics["allan_sampling"] = require_allan_sampling(
                    [s["stamp_ns"] for s in observations])
        result[topic] = metrics
    if include_camera:
        left = [s["stamp_ns"] for s in samples[rig["camera"]["left_image_topic"]]]
        right = [s["stamp_ns"] for s in samples[rig["camera"]["right_image_topic"]]]
        tolerance = rig["camera"]["pair_tolerance_s"] * 1e9
        # Greedy sorted one-to-one matching; don't count one frame more than once.
        i = j = matched = 0
        while i < len(left) and j < len(right):
            if abs(left[i] - right[j]) <= tolerance:
                matched += 1
                i += 1
                j += 1
            elif left[i] < right[j]:
                i += 1
            else:
                j += 1
        ratio = matched / max(len(left), len(right))
        if ratio < .95:
            raise ValueError(f"stereo pair ratio below 95%: {ratio}")
        result["stereo_pair_ratio"] = ratio
    return result


class SensorProbe:
    def __init__(self, rig, include_camera, include_imu, allan_sampling=False):
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray
        from sensor_msgs.msg import Image, CameraInfo, Imu
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.rclpy = rclpy
        self.rig = rig
        self.camera = include_camera
        self.imu = include_imu
        self.allan_sampling = allan_sampling
        self.samples = {}
        self.sync = {}
        self.subscriptions = []
        self.node = rclpy.create_node("calibration_preflight")
        if include_camera:
            for key, msg_type in [("left_image_topic", Image), ("right_image_topic", Image),
                                  ("left_info_topic", CameraInfo), ("right_info_topic", CameraInfo)]:
                topic = rig["camera"][key]
                self.samples[topic] = deque(maxlen=3000)
                self.subscriptions.append(self.node.create_subscription(
                    msg_type, topic, lambda msg, t=topic: self.observe(t, msg),
                    QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)))
        if include_imu:
            topic = rig["imu"]["topic"]
            self.samples[topic] = deque(maxlen=3000)
            self.subscriptions.append(self.node.create_subscription(
                Imu, topic, lambda msg, t=topic: self.observe(t, msg),
                QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)))
            self.subscriptions.append(self.node.create_subscription(
                DiagnosticArray, "/spr/diagnostics", self.diagnostic,
                QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)))

    def observe(self, topic, msg):
        stamp_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        data = {"stamp_ns": stamp_ns, "received_ns": self.node.get_clock().now().nanoseconds,
                "frame_id": msg.header.frame_id}
        if hasattr(msg, "width"):
            data.update(width=msg.width, height=msg.height)
            if hasattr(msg, "encoding"):
                data["valid"] = (msg.encoding == "mono8" and msg.step >= msg.width and
                                 len(msg.data) == msg.step * msg.height)
            else:
                data["valid"] = all(math.isfinite(x) for x in msg.k) and msg.k[0] > 0 and msg.k[4] > 0
        else:
            values = [getattr(v, axis) for v in (msg.angular_velocity, msg.linear_acceleration)
                      for axis in ("x", "y", "z")]
            data["valid"] = all(math.isfinite(x) for x in values)
        self.samples[topic].append(data)

    def diagnostic(self, msg):
        for status in msg.status:
            if self.rig.get("calibration_profile") == "mavlink_calibration":
                if status.name == "drone_bringup/mavlink_imu":
                    from .mavlink_contract import diagnostic_snapshot
                    self.sync = diagnostic_snapshot(status, self.node.get_clock().now().nanoseconds)
            elif "px4_bridge" in status.name:
                self.sync = {v.key: v.value for v in status.values}
                self.sync.update(level=status.level, message=status.message,
                                 received_at_ns=self.node.get_clock().now().nanoseconds)

    def spin(self, seconds=.05):
        self.rclpy.spin_once(self.node, timeout_sec=seconds)

    def camera_configuration(self):
        """Read back driver parameters and installed version, not just requests."""
        if not self.camera:
            return None
        from ament_index_python.packages import get_package_share_directory
        from pathlib import Path
        from rcl_interfaces.msg import ParameterType
        from rcl_interfaces.srv import GetParameters
        import xml.etree.ElementTree as ET
        from .config import camera_parameters
        expected = camera_parameters(self.rig)
        names = ["serial_no", "depth_module.infra_profile", "depth_module.exposure",
                 "depth_module.gain", "depth_module.enable_auto_exposure",
                 "depth_module.emitter_enabled", "enable_depth", "enable_color",
                 "enable_gyro", "enable_accel", "enable_infra1", "enable_infra2"]
        client = self.node.create_client(GetParameters, "/camera/get_parameters")
        try:
            if not client.wait_for_service(timeout_sec=3.0):
                raise ValueError("camera parameter service is unavailable")
            request = GetParameters.Request()
            request.names = names
            future = client.call_async(request)
            self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            if not future.done() or future.result() is None:
                raise ValueError("camera parameter snapshot timed out")
            values = future.result().values
            if len(values) != len(names):
                raise ValueError("camera parameter snapshot is incomplete")
            fields = {ParameterType.PARAMETER_BOOL: "bool_value",
                      ParameterType.PARAMETER_INTEGER: "integer_value",
                      ParameterType.PARAMETER_DOUBLE: "double_value",
                      ParameterType.PARAMETER_STRING: "string_value"}
            observed = {}
            for name, value in zip(names, values):
                if value.type not in fields:
                    raise ValueError(f"camera parameter {name} is unset or unsupported")
                observed[name] = getattr(value, fields[value.type])
            mismatch = {name: {"requested": expected[name], "observed": observed[name]}
                        for name in names if expected[name] != observed[name]}
            package = Path(get_package_share_directory("realsense2_camera")) / "package.xml"
            version = ET.parse(package).getroot().findtext("version")
            if version != self.rig["camera"]["driver_version"]:
                mismatch["driver_version"] = {"requested": self.rig["camera"]["driver_version"],
                                              "observed": version}
            if mismatch:
                raise ValueError(f"camera requested/observed configuration differs: {mismatch}")
            return {"verified": True, "captured_at_ns": self.node.get_clock().now().nanoseconds,
                    "node": "/camera", "driver_version": version, "parameters": observed}
        finally:
            self.node.destroy_client(client)

    def validate(self):
        now = self.node.get_clock().now().nanoseconds
        # Evaluate only recent data while preserving the previous-sample boundary.
        recent = {}
        for topic, values in self.samples.items():
            recent[topic] = [v for v in values if v["received_ns"] >= now - 5_000_000_000]
            endpoints = self.node.get_publishers_info_by_topic(topic)
            if len(endpoints) != 1:
                raise ValueError(f"{topic}: expected exactly one publisher, got {len(endpoints)}")
            if topic == self.rig["imu"]["topic"]:
                expected_node = ("calibration_mavlink_imu" if self.rig.get("calibration_profile") ==
                                 "mavlink_calibration" else "px4_bridge")
                if endpoints[0].node_name != expected_node:
                    raise ValueError(f"{topic}: source is not {expected_node}")
        result = validate_streams(recent, self.rig, now, self.camera, self.imu,
                                  allan_sampling=self.allan_sampling)
        if self.imu:
            if not self.sync or self.sync["level"] != 0:
                raise ValueError("no healthy selected IMU source time diagnostic")
            if now - self.sync["received_at_ns"] > 2_000_000_000:
                raise ValueError("stale selected IMU source time diagnostic")
            result["imu_sync"] = self.sync
        return result

    def fresh_imu_summary(self, timeout_s=4.0):
        after_ns = self.node.get_clock().now().nanoseconds
        deadline = time.monotonic() + timeout_s
        error = "waiting for a new synchronized IMU diagnostic"
        while time.monotonic() < deadline:
            self.spin(.02)
            if self.sync.get("received_at_ns", 0) <= after_ns:
                continue
            try:
                summary = self.validate()
                from .mavlink_contract import mavlink_provenance
                mavlink_provenance(self.rig, {}, summary["imu_sync"])
                return summary
            except (ValueError, KeyError) as exception:
                error = str(exception)
        raise ValueError("fresh IMU synchronization failed after camera RPC: " + error)

    def close(self):
        self.node.destroy_node()
