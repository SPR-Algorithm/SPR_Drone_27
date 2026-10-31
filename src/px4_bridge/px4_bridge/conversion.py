"""Pure conversion logic; timestamps are integer ns to preserve epoch precision.

PX4 1.16 uXRCE-DDS has already mapped outgoing timestamps to agent time.
The TimesyncStatus estimated_offset has the opposite sign (PX4 minus agent).
It is observed for validity and continuity, never added to incoming samples.
"""

from collections import deque
from dataclasses import dataclass
import math


RELATIVE_TIMESTAMP_INVALID = 2_147_483_647
PX4_MSGS_REVISION = '392e831c1f659429ca83902e66820d7094591410'


class SampleRejected(ValueError):
    """Input cannot be represented faithfully as a common-time IMU sample."""


def frd_to_flu(vector):
    if len(vector) != 3 or not all(math.isfinite(float(v)) for v in vector):
        raise SampleRejected('nonfinite_or_invalid_vector')
    return (float(vector[0]), -float(vector[1]), -float(vector[2]))


def diagonal_covariance(variance):
    """ROS IMU: all zeros mean covariance unknown; -1 is for absent fields.

    Config uses three -1 values for unknown variance. Partial unknown or zero
    specified variance is rejected, so an accidental zero never means certainty.
    """
    if len(variance) != 3:
        raise ValueError('variance must contain three diagonal entries')
    if all(float(v) == -1.0 for v in variance):
        return (0.0,) * 9
    if not all(math.isfinite(float(v)) and float(v) > 0.0 for v in variance):
        raise ValueError('variance must be three positive values or three -1 values')
    return (float(variance[0]), 0.0, 0.0,
            0.0, float(variance[1]), 0.0,
            0.0, 0.0, float(variance[2]))


class TimeSyncGate:
    """Consecutive DDS timesync quality gate, reset for every continuous segment.

    uncertainty_us = RTT/2 + abs(observed_offset - estimated_offset). This is
    an observable conservative packet/filter budget, not ground-truth accuracy.
    """

    def __init__(self, min_samples=20, max_uncertainty_us=1000.0,
                 timeout_s=2.5, max_offset_step_us=1000.0,
                 max_clock_step_s=0.01, max_transport_age_s=0.2):
        values = [min_samples, max_uncertainty_us, timeout_s, max_offset_step_us,
                  max_clock_step_s, max_transport_age_s]
        if (not all(math.isfinite(float(value)) for value in values) or
                min_samples < 1 or int(min_samples) != min_samples or max_uncertainty_us <= 0 or
                timeout_s <= 0 or
                max_offset_step_us <= 0 or max_clock_step_s <= 0 or
                max_transport_age_s <= 0):
            raise ValueError('timesync sample count and finite positive limits are required')
        self.min_samples = int(min_samples)
        self.max_uncertainty_us = float(max_uncertainty_us)
        self.timeout_ns = round(timeout_s * 1e9)
        self.max_offset_step_us = float(max_offset_step_us)
        self.max_clock_step_ns = round(max_clock_step_s * 1e9)
        self.max_age_ns = round(max_transport_age_s * 1e9)
        self.segment = 0
        self.last_ros_ns = None
        self.last_mono_ns = None
        self.reset('waiting_for_dds_timesync')

    def reset(self, reason):
        self.segment += 1
        self.reason = reason
        self.samples = 0
        self.last_status_mono_ns = None
        self.last_status_stamp_us = None
        self.last_boot_stamp_us = None
        self.estimated_offset_us = None
        self.initial_offset_us = None
        self.delta_offset_us = None
        self.uncertainty_us = None
        self.round_trip_time_us = None
        self.ready = False

    def tick(self, ros_ns, mono_ns):
        if self.last_ros_ns is not None:
            ros_delta = ros_ns - self.last_ros_ns
            mono_delta = mono_ns - self.last_mono_ns
            if abs(ros_delta - mono_delta) > self.max_clock_step_ns:
                self.reset('host_clock_jump')
        self.last_ros_ns = ros_ns
        self.last_mono_ns = mono_ns
        if (self.last_status_mono_ns is not None and
                mono_ns - self.last_status_mono_ns > self.timeout_ns):
            self.reset('timesync_timeout')
        return self.ready

    def observe(self, status, ros_ns, mono_ns):
        self.tick(ros_ns, mono_ns)
        if int(status.source_protocol) != 2:
            self.reset('timesync_not_dds')
            return False
        stamp_us = int(status.timestamp)
        offset_us = int(status.estimated_offset)
        # DDS rewrites timestamp, but not remote_timestamp or offset fields.
        boot_us = stamp_us + offset_us
        if (stamp_us <= 0 or boot_us < 0 or
                abs(stamp_us * 1000 - ros_ns) > self.max_age_ns or
                abs(int(status.remote_timestamp) * 1000 - ros_ns) > self.max_age_ns):
            self.reset('timesync_wrong_time_domain_or_stale')
            return False
        if (self.last_status_stamp_us is not None and
                (stamp_us <= self.last_status_stamp_us or
                 boot_us <= self.last_boot_stamp_us)):
            self.reset('timesync_nonmonotonic_or_px4_restart')
            return False
        uncertainty = (int(status.round_trip_time) / 2.0 +
                       abs(int(status.observed_offset) - offset_us))
        if uncertainty > self.max_uncertainty_us:
            self.reset('timesync_uncertainty_exceeded')
            self.uncertainty_us = uncertainty
            return False
        if (self.estimated_offset_us is not None and
                abs(offset_us - self.estimated_offset_us) > self.max_offset_step_us):
            self.reset('timesync_offset_jump')
            return False
        if self.initial_offset_us is None:
            self.initial_offset_us = offset_us
        self.estimated_offset_us = offset_us
        # host-minus-PX4 sign; informational only, already applied by firmware.
        self.delta_offset_us = self.initial_offset_us - offset_us
        self.uncertainty_us = uncertainty
        self.round_trip_time_us = int(status.round_trip_time)
        self.last_status_stamp_us = stamp_us
        self.last_boot_stamp_us = boot_us
        self.last_status_mono_ns = mono_ns
        self.samples += 1
        self.ready = self.samples >= self.min_samples
        self.reason = 'synchronized' if self.ready else 'timesync_converging'
        return self.ready

    def validate_sample(self, stamp_ns, ros_ns, mono_ns):
        if not self.tick(ros_ns, mono_ns):
            raise SampleRejected(self.reason)
        age_ns = ros_ns - stamp_ns
        if age_ns < -self.max_uncertainty_us * 1000 or age_ns > self.max_age_ns:
            self.reset('imu_wrong_time_domain_or_stale')
            raise SampleRejected(self.reason)
        return stamp_ns  # identity: do not apply the DDS offset a second time


@dataclass(frozen=True)
class ImuSample:
    timestamp_ns: int
    angular_velocity: tuple
    linear_acceleration: tuple
    interpolation_span_ns: int


class ImuAssembler:
    """Interpolate acceleration to gyro integration midpoints, no extrapolation.

    SensorCombined.timestamp is the gyro integration END in PX4 v1.16. It has
    no timestamp_sample member. Acceleration can have a distinct end timestamp.
    The integration interval is subtracted (half), not added.
    """

    def __init__(self, interpolation_max_gap_s=0.02, max_gap_s=0.1):
        if (not math.isfinite(interpolation_max_gap_s) or not math.isfinite(max_gap_s) or
                interpolation_max_gap_s <= 0 or max_gap_s <= 0):
            raise ValueError('IMU gap limits must be positive')
        self.interpolation_max_gap_ns = round(interpolation_max_gap_s * 1e9)
        self.max_gap_ns = round(max_gap_s * 1e9)
        self.reset()

    def reset(self):
        self.accels = deque(maxlen=64)
        self.gyros = deque(maxlen=64)
        self.last_input_ns = None
        self.last_output_ns = None
        self.calibration_counts = None
        self.dropped_unbracketed = 0

    @staticmethod
    def gyro_midpoint_ns(message):
        return int(message.timestamp) * 1000 - int(message.gyro_integral_dt) * 500

    def add(self, message):
        if int(message.accelerometer_timestamp_relative) == RELATIVE_TIMESTAMP_INVALID:
            raise SampleRejected('accelerometer_relative_timestamp_invalid')
        if int(message.accelerometer_clipping) or int(message.gyro_clipping):
            raise SampleRejected('imu_clipping')
        gyro_dt = int(message.gyro_integral_dt)
        accel_dt = int(message.accelerometer_integral_dt)
        if gyro_dt <= 0 or accel_dt <= 0:
            raise SampleRejected('imu_invalid_integration_interval')
        if max(gyro_dt, accel_dt) * 1000 > self.max_gap_ns:
            raise SampleRejected('imu_integration_interval_too_long')
        endpoint_ns = int(message.timestamp) * 1000
        gyro_ns = endpoint_ns - gyro_dt * 500
        accel_ns = (endpoint_ns + int(message.accelerometer_timestamp_relative) * 1000
                    - accel_dt * 500)
        if min(gyro_ns, accel_ns) <= 0:
            raise SampleRejected('imu_nonpositive_sample_time')
        if abs(gyro_ns - accel_ns) > self.interpolation_max_gap_ns:
            raise SampleRejected('imu_sensor_skew_exceeded')
        gyro = frd_to_flu(message.gyro_rad)
        accel = frd_to_flu(message.accelerometer_m_s2)
        counts = (int(message.gyro_calibration_count), int(message.accel_calibration_count))
        if self.calibration_counts is not None and counts != self.calibration_counts:
            self.reset()
            raise SampleRejected('imu_calibration_changed')
        self.calibration_counts = counts
        if self.last_input_ns is not None:
            if gyro_ns <= self.last_input_ns:
                self.reset()
                raise SampleRejected('imu_timestamp_nonmonotonic')
            if gyro_ns - self.last_input_ns > self.max_gap_ns:
                self.reset()
                raise SampleRejected('imu_sample_gap')
        self.last_input_ns = gyro_ns
        if self.accels and accel_ns < self.accels[-1][0]:
            self.reset()
            raise SampleRejected('accel_timestamp_nonmonotonic')
        if not self.accels or accel_ns != self.accels[-1][0]:
            self.accels.append((accel_ns, accel))
        if len(self.gyros) == self.gyros.maxlen:
            raise SampleRejected('imu_pending_queue_overflow')
        self.gyros.append((gyro_ns, gyro))
        result = []
        while self.gyros and self.accels:
            target_ns, gyro = self.gyros[0]
            if self.accels[0][0] > target_ns:
                self.gyros.popleft()
                self.dropped_unbracketed += 1
                continue
            while len(self.accels) > 1 and self.accels[1][0] <= target_ns:
                self.accels.popleft()
            left_ns, left = self.accels[0]
            span = 0
            if left_ns == target_ns:
                interpolated = left
            elif len(self.accels) > 1:
                right_ns, right = self.accels[1]
                span = right_ns - left_ns
                if span > self.interpolation_max_gap_ns:
                    self.gyros.popleft()
                    self.dropped_unbracketed += 1
                    continue
                fraction = (target_ns - left_ns) / span
                interpolated = tuple(a + fraction * (b - a) for a, b in zip(left, right))
            else:
                break  # wait for a future accelerometer sample; never extrapolate
            self.gyros.popleft()
            if self.last_output_ns is not None and target_ns <= self.last_output_ns:
                raise SampleRejected('imu_output_nonmonotonic')
            self.last_output_ns = target_ns
            result.append(ImuSample(target_ns, gyro, interpolated, span))
        return result
