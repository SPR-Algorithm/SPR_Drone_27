"""ROS-free bag inspection/conversion; headers and all semantic payloads are checked."""

import hashlib
import json
from pathlib import Path
import struct

import numpy as np

from .common import CalibrationError, hash_input, tool_versions


def _imports():
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as error:
        raise CalibrationError("Install requirements-tools.txt in a Python3 venv; rosbags is missing") from error
    return AnyReader, get_typestore(Stores.ROS2_JAZZY)


def messages(path, topics):
    AnyReader, store = _imports()
    with AnyReader([Path(path)], default_typestore=store) as reader:
        selected = [connection for connection in reader.connections if connection.topic in topics]
        found = {connection.topic for connection in selected}
        if found != set(topics):
            raise CalibrationError(f"Missing bag topics: {sorted(set(topics) - found)}")
        for connection in selected:
            if connection.msgtype != topics[connection.topic]:
                raise CalibrationError(f"{connection.topic}: expected {topics[connection.topic]}, got {connection.msgtype}")
        for connection, recorded, raw in reader.messages(connections=selected):
            yield connection.topic, recorded, reader.deserialize(raw, connection.msgtype)


def header_ns(message):
    stamp = message.header.stamp
    ns = int(stamp.sec) * 10**9 + int(stamp.nanosec)
    if ns <= 0 or not 0 <= int(stamp.nanosec) < 10**9:
        raise CalibrationError("Header timestamp must be a positive valid ROS timestamp")
    return ns


def payload_digest(message):
    """Hash every field used by calibration, including frame/covariances and pixels."""
    digest = hashlib.sha256()
    digest.update(str(message.header.frame_id).encode())
    if hasattr(message, "encoding"):
        digest.update(json.dumps([message.width, message.height, message.encoding,
                                  message.is_bigendian, message.step]).encode())
        digest.update(message.data.tobytes())
    else:
        for attr, axes in (("orientation", "xyzw"), ("angular_velocity", "xyz"), ("linear_acceleration", "xyz")):
            vector = getattr(message, attr)
            values = [float(getattr(vector, axis)) for axis in axes]
            if not np.isfinite(values).all():
                raise CalibrationError(f"IMU {attr} contains nonfinite values")
            digest.update(struct.pack("<" + "d" * len(values), *values))
            covariance = np.asarray(getattr(message, attr + "_covariance"), dtype="<f8")
            if not np.isfinite(covariance).all():
                raise CalibrationError(f"IMU {attr} covariance contains nonfinite values")
            digest.update(covariance.tobytes())
    return digest.digest()


def inspect_bag(path, topics):
    result = {topic: {"type": msgtype, "count": 0} for topic, msgtype in topics.items()}
    digests = {topic: hashlib.sha256() for topic in topics}
    payload_digests = {topic: hashlib.sha256() for topic in topics}
    stamps = {topic: [] for topic in topics}
    frames = {topic: set() for topic in topics}
    shapes = {topic: set() for topic in topics}
    for topic, recorded, message in messages(path, topics):
        ns = header_ns(message)
        if stamps[topic] and ns <= stamps[topic][-1]:
            raise CalibrationError(f"{topic}: repeated or backward header timestamp at message {len(stamps[topic])}")
        stamps[topic].append(ns)
        frames[topic].add(str(message.header.frame_id))
        if hasattr(message, "encoding"):
            shape = (int(message.width), int(message.height), str(message.encoding), int(message.step))
            if message.encoding not in ("mono8", "8UC1") or message.step < message.width or len(message.data) != message.step * message.height:
                raise CalibrationError(f"{topic}: expected a valid uncompressed Y8/mono8 image")
            shapes[topic].add(shape)
        digests[topic].update(struct.pack("<qq", int(recorded), ns))
        payload = payload_digest(message)
        digests[topic].update(payload)
        payload_digests[topic].update(payload)
        result[topic]["count"] += 1
    for topic, item in result.items():
        if item["count"] < 2:
            raise CalibrationError(f"{topic}: at least two messages required, got {item['count']}")
        times = np.asarray(stamps[topic], dtype=np.int64)
        dt = np.diff(times).astype(float) / 1e9
        if len(frames[topic]) != 1 or not next(iter(frames[topic])):
            raise CalibrationError(f"{topic}: frame_id must be stable and nonempty")
        if len(shapes[topic]) > 1:
            raise CalibrationError(f"{topic}: changing image profile")
        item.update(first_header_ns=int(times[0]), last_header_ns=int(times[-1]),
                    duration_s=float((times[-1] - times[0]) / 1e9),
                    rate_hz=float((len(times) - 1) / ((times[-1] - times[0]) / 1e9)),
                    median_rate_hz=float(1 / np.median(dt)), interval_p99_s=float(np.quantile(dt, 0.99)),
                    max_interval_s=float(np.max(dt)), frame_id=next(iter(frames[topic])),
                    semantic_sha256=digests[topic].hexdigest(),
                    ordered_payload_sha256=payload_digests[topic].hexdigest())
        if shapes[topic]:
            item["profile"] = list(next(iter(shapes[topic])))
    return {"input_sha256": hash_input(path), "topics": result}


def convert_bag(source, destination, topics):
    """Convert only selected standard messages; verify all messages after conversion."""
    from rosbags.rosbag1 import Writer
    from rosbags.typesys import Stores, get_typestore
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if source.is_dir() and destination.is_relative_to(source):
        raise CalibrationError("Conversion output cannot be inside the source bag")
    if destination.exists():
        raise CalibrationError(f"Refusing to overwrite {destination}")
    before = inspect_bag(source, topics)
    store = get_typestore(Stores.ROS1_NOETIC)
    # AnyReader normalizes ROS1/2 type names. ROS1 alone has Header.seq;
    # introduce zero only when the source ROS2 message has no sequence field.
    try:
        with Writer(destination) as writer:
            connections = {topic: writer.add_connection(topic, msgtype, typestore=store)
                           for topic, msgtype in topics.items()}
            for topic, recorded, message in messages(source, topics):
                original = message.header
                message.header = store.types["std_msgs/msg/Header"](
                    seq=int(getattr(original, "seq", 0)), stamp=original.stamp,
                    frame_id=original.frame_id)
                writer.write(connections[topic], recorded,
                             store.serialize_ros1(message, topics[topic]))
        after = inspect_bag(destination, topics)
        if before["topics"] != after["topics"]:
            raise CalibrationError("Conversion changed message counts, timestamps or payload hashes")
    except Exception:
        destination.unlink(missing_ok=True)
        raise
    return {"source": before, "converted": after, "verified_all_messages": True,
            "converter_versions": tool_versions()}


def load_imu(path, topic="/spr/imu"):
    times, values = [], []
    for _, _, message in messages(path, {topic: "sensor_msgs/msg/Imu"}):
        times.append(header_ns(message))
        values.append([getattr(vector, axis) for vector in
                       (message.angular_velocity, message.linear_acceleration) for axis in "xyz"])
    ns = np.asarray(times, dtype=np.int64)
    if len(ns) < 3 or np.any(np.diff(ns) <= 0):
        raise CalibrationError("IMU timestamps must be strictly increasing; at least three samples required")
    values = np.asarray(values, dtype=float)
    if not np.isfinite(values).all():
        raise CalibrationError("Nonfinite IMU samples")
    return ns, values
