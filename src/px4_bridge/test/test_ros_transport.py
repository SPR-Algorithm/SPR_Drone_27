"""Real ROS/DDS conversion test; no hardware or flight-control publishers.

Run after sourcing the pinned px4_msgs workspace. A skip without this dependency
does not establish transport acceptance; pure conversion tests still run.
"""

import os
import time

import pytest


pytestmark = pytest.mark.skipif(
    os.environ.get('SPR_ALLOW_SYNTHETIC_ROS_TESTS') != '1',
    reason='synthetic ROS publishers require explicit opt-in; never use for hardware acceptance',
)


rclpy = pytest.importorskip('rclpy')
px4_messages = pytest.importorskip('px4_msgs.msg')
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from px4_bridge.node import Px4ImuBridge


def spin_until(executor, predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
        if predicate():
            return True
    return False


@pytest.fixture
def ros_pair():
    context = Context()
    rclpy.init(context=context, domain_id=182)
    suffix = f'/spr_bridge_test_{time.monotonic_ns()}'
    topic_map = {
        'sensor_topic': suffix + '/input',
        'timesync_topic': suffix + '/time',
        'imu_topic': suffix + '/imu',
        'diagnostics_topic': suffix + '/diagnostics',
    }
    overrides = [Parameter(name, value=value) for name, value in topic_map.items()]
    overrides.extend([
        Parameter('sync_min_samples', value=2),
        Parameter('max_gap_s', value=0.5),
    ])
    bridge = Px4ImuBridge(context=context, parameter_overrides=overrides)
    probe = Node('bridge_transport_probe', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(bridge)
    executor.add_node(probe)
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    sensor_publisher = probe.create_publisher(px4_messages.SensorCombined, topic_map['sensor_topic'], qos)
    sync_publisher = probe.create_publisher(px4_messages.TimesyncStatus, topic_map['timesync_topic'], qos)
    samples = []
    subscription = probe.create_subscription(Imu, topic_map['imu_topic'], samples.append, qos)
    assert spin_until(executor, lambda: sensor_publisher.get_subscription_count() > 0 and
                      sync_publisher.get_subscription_count() > 0 and
                      subscription.get_publisher_count() > 0)
    try:
        yield bridge, executor, sensor_publisher, sync_publisher, samples
    finally:
        executor.remove_node(probe)
        executor.remove_node(bridge)
        probe.destroy_node()
        bridge.destroy_node()
        executor.shutdown()
        context.shutdown()


def publish_timesync(executor, publisher, bridge, *, protocol=2, rtt=200):
    stamp_us = time.time_ns() // 1000
    offset_us = -(stamp_us - 10_000_000)
    message = px4_messages.TimesyncStatus()
    message.timestamp = stamp_us
    message.remote_timestamp = stamp_us
    message.estimated_offset = offset_us
    message.observed_offset = offset_us
    message.round_trip_time = rtt
    message.source_protocol = protocol
    previous_segment = bridge.sync.segment
    publisher.publish(message)
    assert spin_until(executor, lambda: bridge.sync.last_status_stamp_us == stamp_us or
                      bridge.sync.segment != previous_segment)


def make_sensor(stamp_us):
    message = px4_messages.SensorCombined()
    message.timestamp = stamp_us
    message.gyro_rad = [1.0, 2.0, 3.0]
    message.gyro_integral_dt = 5000
    message.accelerometer_integral_dt = 5000
    message.accelerometer_m_s2 = [0.0, 0.0, -9.81]
    return message


def test_real_messages_transport_and_fails_closed_after_bad_sync(ros_pair):
    bridge, executor, sensor_publisher, sync_publisher, samples = ros_pair
    # Raw delivery alone may not substitute receipt time for missing sync.
    sensor_publisher.publish(make_sensor(time.time_ns() // 1000))
    assert spin_until(executor, lambda: sum(bridge.drops.values()) > 0)
    assert samples == []
    publish_timesync(executor, sync_publisher, bridge)
    publish_timesync(executor, sync_publisher, bridge)
    assert bridge.sync.ready
    stamp_us = time.time_ns() // 1000
    sensor_publisher.publish(make_sensor(stamp_us))
    assert spin_until(executor, lambda: len(samples) == 1)
    result = samples[0]
    assert result.header.frame_id == 'imu_link'
    assert result.header.stamp.sec * 1_000_000_000 + result.header.stamp.nanosec == stamp_us * 1000 - 2_500_000
    assert result.angular_velocity.x == 1.0
    assert result.angular_velocity.y == -2.0
    assert result.angular_velocity.z == -3.0
    assert result.linear_acceleration.z == pytest.approx(9.81, abs=1e-5)
    assert result.orientation_covariance[0] == -1.0
    assert list(result.angular_velocity_covariance) == [0.0] * 9
    assert list(result.linear_acceleration_covariance) == [0.0] * 9
    publish_timesync(executor, sync_publisher, bridge, rtt=3000)
    assert not bridge.sync.ready
    before = sum(bridge.drops.values())
    sensor_publisher.publish(make_sensor(time.time_ns() // 1000))
    assert spin_until(executor, lambda: sum(bridge.drops.values()) > before)
    assert len(samples) == 1


def test_real_messages_reconnection_requires_new_time_segment(ros_pair):
    bridge, executor, sensor_publisher, sync_publisher, samples = ros_pair
    publish_timesync(executor, sync_publisher, bridge)
    publish_timesync(executor, sync_publisher, bridge)
    sensor_publisher.publish(make_sensor(time.time_ns() // 1000))
    assert spin_until(executor, lambda: len(samples) == 1)
    segment = bridge.sync.segment
    assert spin_until(executor, lambda: bridge.sync.reason == 'imu_link_timeout', timeout=1.5)
    assert bridge.sync.segment > segment
    assert not bridge.sync.ready
    publish_timesync(executor, sync_publisher, bridge)
    assert not bridge.sync.ready
    publish_timesync(executor, sync_publisher, bridge)
    assert bridge.sync.ready
    sensor_publisher.publish(make_sensor(time.time_ns() // 1000))
    assert spin_until(executor, lambda: len(samples) == 2)


def test_reject_sim_clock_until_agent_sim_time_mapping_exists():
    context = Context()
    rclpy.init(context=context, domain_id=183)
    try:
        with pytest.raises(ValueError, match='use_sim_time=false'):
            Px4ImuBridge(context=context, parameter_overrides=[Parameter('use_sim_time', value=True)])
    finally:
        context.shutdown()
