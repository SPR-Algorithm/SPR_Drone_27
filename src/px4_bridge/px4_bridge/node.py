"""ROS transport for the read-only A0 IMU bridge; no device or control access."""

from collections import Counter
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from px4_msgs.msg import SensorCombined, TimesyncStatus
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from .conversion import (
    ImuAssembler, PX4_MSGS_REVISION, SampleRejected, TimeSyncGate, diagonal_covariance,
)


DEFAULTS = {
    'sensor_topic': '/fmu/out/sensor_combined',
    'timesync_topic': '/fmu/out/timesync_status',
    'imu_topic': '/spr/imu',
    'diagnostics_topic': '/spr/diagnostics',
    'imu_frame_id': 'imu_link',
    'input_time_domain': 'agent_synced',
    'timestamp_source': 'integration_midpoint',
    'sync_min_samples': 20,
    'sync_max_uncertainty_us': 1000.0,
    'sync_timeout_s': 2.5,
    'sync_max_offset_step_us': 1000.0,
    'max_clock_step_s': 0.01,
    'max_transport_age_s': 0.2,
    'max_gap_s': 0.1,
    'interpolation_max_gap_s': 0.02,
    'min_rate_hz': 200.0,
    'gyro_variance': [-1.0, -1.0, -1.0],
    'accel_variance': [-1.0, -1.0, -1.0],
}


class Px4ImuBridge(Node):
    def __init__(self, **kwargs):
        super().__init__('px4_bridge', **kwargs)
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        self.config = {name: self.get_parameter(name).value for name in DEFAULTS}
        cfg = self.config
        if cfg['input_time_domain'] != 'agent_synced':
            raise ValueError('PX4 1.16 UXRCE_DDS_SYNCT=1 required; input_time_domain=agent_synced')
        if cfg['timestamp_source'] != 'integration_midpoint':
            raise ValueError('timestamp_source must be integration_midpoint')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('real-agent time mapping requires use_sim_time=false; replay /spr/imu directly')
        if not cfg['imu_frame_id'] or cfg['min_rate_hz'] <= 0:
            raise ValueError('imu_frame_id and positive min_rate_hz are required')
        self.gyro_covariance = list(diagonal_covariance(cfg['gyro_variance']))
        self.accel_covariance = list(diagonal_covariance(cfg['accel_variance']))
        self.sync = TimeSyncGate(
            min_samples=cfg['sync_min_samples'],
            max_uncertainty_us=cfg['sync_max_uncertainty_us'],
            timeout_s=cfg['sync_timeout_s'],
            max_offset_step_us=cfg['sync_max_offset_step_us'],
            max_clock_step_s=cfg['max_clock_step_s'],
            max_transport_age_s=cfg['max_transport_age_s'],
        )
        self.assembler = ImuAssembler(
            interpolation_max_gap_s=cfg['interpolation_max_gap_s'],
            max_gap_s=cfg['max_gap_s'],
        )
        self.segment = self.sync.segment
        self.last_input_mono_ns = None
        self.last_output_ns = None
        self.output_count = 0
        self.window_output_count = 0
        self.window_start_ns = time.monotonic_ns()
        self.rate_hz = 0.0
        self.drops = Counter()
        self.last_event = 'waiting_for_dds_timesync'
        sensor_qos = QoSProfile(
            depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        diagnostic_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.imu_publisher = self.create_publisher(Imu, cfg['imu_topic'], sensor_qos)
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, cfg['diagnostics_topic'], diagnostic_qos)
        self.sensor_subscription = self.create_subscription(
            SensorCombined, cfg['sensor_topic'], self._sensor_callback, sensor_qos)
        self.sync_subscription = self.create_subscription(
            TimesyncStatus, cfg['timesync_topic'], self._timesync_callback, sensor_qos)
        self.add_on_set_parameters_callback(self._reject_runtime_reconfiguration)
        self.watchdog = self.create_timer(0.05, self._watchdog)
        self.diagnostic_timer = self.create_timer(1.0, self._publish_diagnostics)
        self.get_logger().info(
            f'Read-only SensorCombined bridge: FRD -> {cfg["imu_frame_id"]} FLU; '
            'agent-synced timestamps, integration midpoints; waiting for DDS timesync quality gate')

    @staticmethod
    def _reject_runtime_reconfiguration(_parameters):
        return SetParametersResult(
            successful=False, reason='PX4 IMU configuration is fixed for a continuous run; restart via bringup')

    def _now(self):
        return self.get_clock().now().nanoseconds, time.monotonic_ns()

    def _apply_segment_change(self):
        if self.segment != self.sync.segment:
            self.segment = self.sync.segment
            self.assembler.reset()
            self.last_output_ns = None
            self.window_output_count = 0
            self.rate_hz = 0.0
            self.window_start_ns = time.monotonic_ns()
            self._event(self.sync.reason)

    def _event(self, reason):
        if reason != self.last_event:
            self.last_event = reason
            self.get_logger().warning(f'IMU continuity event: {reason}; segment={self.segment}')
            self._publish_diagnostics(update_rate=False)

    def _timesync_callback(self, status):
        ros_ns, mono_ns = self._now()
        self.sync.observe(status, ros_ns, mono_ns)
        self._apply_segment_change()
        if self.sync.ready and self.last_event != 'synchronized':
            self.last_event = 'synchronized'
            self.get_logger().info(f'DDS timesync ready; segment={self.segment}')
            self._publish_diagnostics(update_rate=False)

    def _sensor_callback(self, message):
        ros_ns, mono_ns = self._now()
        self.last_input_mono_ns = mono_ns
        try:
            self.sync.validate_sample(
                self.assembler.gyro_midpoint_ns(message), ros_ns, mono_ns)
            self._apply_segment_change()
            for sample in self.assembler.add(message):
                output = Imu()
                output.header.frame_id = self.config['imu_frame_id']
                output.header.stamp.sec = sample.timestamp_ns // 1_000_000_000
                output.header.stamp.nanosec = sample.timestamp_ns % 1_000_000_000
                # SensorCombined does not measure orientation.
                output.orientation.w = 1.0
                output.orientation_covariance[0] = -1.0
                output.angular_velocity.x, output.angular_velocity.y, output.angular_velocity.z = sample.angular_velocity
                output.linear_acceleration.x, output.linear_acceleration.y, output.linear_acceleration.z = sample.linear_acceleration
                output.angular_velocity_covariance = self.gyro_covariance
                output.linear_acceleration_covariance = self.accel_covariance
                self.imu_publisher.publish(output)
                self.output_count += 1
                self.window_output_count += 1
                self.last_output_ns = sample.timestamp_ns
        except SampleRejected as error:
            reason = str(error)
            self.drops[reason] += 1
            if reason in {
                'imu_timestamp_nonmonotonic', 'imu_calibration_changed',
                'imu_sample_gap', 'accel_timestamp_nonmonotonic',
                'imu_output_nonmonotonic', 'imu_pending_queue_overflow',
            }:
                self.sync.reset(reason)
            self._apply_segment_change()

    def _watchdog(self):
        ros_ns, mono_ns = self._now()
        self.sync.tick(ros_ns, mono_ns)
        if (self.last_input_mono_ns is not None and
                mono_ns - self.last_input_mono_ns > round(self.config['max_gap_s'] * 1e9)):
            self.last_input_mono_ns = None
            self.sync.reset('imu_link_timeout')
        self._apply_segment_change()

    def _publish_diagnostics(self, update_rate=True):
        mono_ns = time.monotonic_ns()
        if update_rate:
            duration_s = (mono_ns - self.window_start_ns) / 1e9
            self.rate_hz = self.window_output_count / duration_s if duration_s > 0 else 0.0
            self.window_output_count = 0
            self.window_start_ns = mono_ns
        status = DiagnosticStatus()
        status.name = 'px4_bridge/imu'
        status.hardware_id = 'px4_sensor_combined'
        if not self.sync.ready or self.last_input_mono_ns is None:
            status.level = DiagnosticStatus.ERROR
            status.message = self.sync.reason if not self.sync.ready else 'waiting_for_imu'
        elif self.rate_hz < self.config['min_rate_hz']:
            status.level = DiagnosticStatus.WARN
            status.message = 'imu_rate_below_target'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'synchronized'
        metadata = {
            'event_type': self.last_event,
            'segment_id': self.segment,
            'sync_ready': self.sync.ready,
            'sync_samples': self.sync.samples,
            'estimated_offset_us': self.sync.estimated_offset_us,
            'host_minus_px4_offset_us': (
                -self.sync.estimated_offset_us if self.sync.estimated_offset_us is not None else None),
            'delta_host_minus_px4_offset_us': self.sync.delta_offset_us,
            'round_trip_time_us': self.sync.round_trip_time_us,
            'sync_uncertainty_us': self.sync.uncertainty_us,
            'sync_budget_us': self.config['sync_max_uncertainty_us'],
            'sync_protocol': 'uxrce_dds_timesync_status',
            'timestamp_mode': 'agent_synced_no_second_offset',
            'timestamp_source': 'gyro_integration_end_minus_half_dt',
            'acceleration_time_policy': 'linear_interpolation_to_gyro_midpoint_no_extrapolation',
            'frame_id': self.config['imu_frame_id'],
            'frame_conversion': 'FRD_to_FLU_diag_1_minus1_minus1',
            'orientation_available': False,
            'gyro_covariance_known': any(self.gyro_covariance),
            'accel_covariance_known': any(self.accel_covariance),
            'rate_hz': self.rate_hz,
            'output_count': self.output_count,
            'last_sample_ns': self.last_output_ns,
            'dropped_unbracketed': self.assembler.dropped_unbracketed,
            'px4_msgs_revision': PX4_MSGS_REVISION,
        }
        metadata.update({f'dropped_{key}': count for key, count in sorted(self.drops.items())})
        status.values = [KeyValue(key=str(key), value=str(value)) for key, value in metadata.items()]
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        output.status = [status]
        self.diagnostic_publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Px4ImuBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
