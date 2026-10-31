from types import SimpleNamespace

import pytest

from px4_bridge.conversion import (
    ImuAssembler, RELATIVE_TIMESTAMP_INVALID, SampleRejected, TimeSyncGate,
    diagonal_covariance, frd_to_flu,
)


EPOCH_US = 1_790_000_000_000_000
OFFSET_US = -EPOCH_US + 1_000_000


def status(stamp_us=EPOCH_US, **kwargs):
    fields = dict(timestamp=stamp_us, source_protocol=2, remote_timestamp=stamp_us,
                  estimated_offset=OFFSET_US, observed_offset=OFFSET_US + 10,
                  round_trip_time=400)
    fields.update(kwargs)
    return SimpleNamespace(**fields)


def sensor(timestamp_us=EPOCH_US, **kwargs):
    fields = dict(timestamp=timestamp_us, gyro_rad=[1.0, 2.0, 3.0],
                  gyro_integral_dt=5000, accelerometer_timestamp_relative=0,
                  accelerometer_m_s2=[4.0, 5.0, -9.81],
                  accelerometer_integral_dt=5000, accelerometer_clipping=0,
                  gyro_clipping=0, gyro_calibration_count=0, accel_calibration_count=0)
    fields.update(kwargs)
    return SimpleNamespace(**fields)


def ready_gate(**kwargs):
    gate = TimeSyncGate(min_samples=2, **kwargs)
    assert not gate.observe(status(), EPOCH_US * 1000, 0)
    assert gate.observe(status(EPOCH_US + 1000), (EPOCH_US + 1000) * 1000, 1_000_000)
    return gate


def test_frd_flu_known_basis_and_specific_force():
    assert frd_to_flu([1, 0, 0]) == (1, 0, 0)
    assert frd_to_flu([0, 1, 0]) == (0, -1, 0)
    assert frd_to_flu([0, 0, 1]) == (0, 0, -1)
    assert frd_to_flu([0, 0, -9.81]) == (0, 0, 9.81)
    with pytest.raises(SampleRejected):
        frd_to_flu([float('nan'), 0, 0])


def test_covariance_unknown_and_measured_variance():
    assert diagonal_covariance([-1, -1, -1]) == (0.0,) * 9
    assert diagonal_covariance([1, 2, 3]) == (1, 0, 0, 0, 2, 0, 0, 0, 3)
    for values in ([0, 0, 0], [-1, 2, 3], [1, 2], [float('nan'), 1, 1]):
        with pytest.raises(ValueError):
            diagonal_covariance(values)


def test_agent_synced_stamp_is_not_offset_twice():
    gate = ready_gate()
    stamp_ns = (EPOCH_US + 1000) * 1000 - 2_500_000
    assert gate.validate_sample(stamp_ns, (EPOCH_US + 1000) * 1000, 1_000_000) == stamp_ns
    assert gate.estimated_offset_us == OFFSET_US
    assert gate.uncertainty_us == 210.0
    assert gate.round_trip_time_us == 400


@pytest.mark.parametrize('changes,reason', [
    ({'source_protocol': 1}, 'timesync_not_dds'),
    ({'round_trip_time': 2002}, 'timesync_uncertainty_exceeded'),
    ({'observed_offset': OFFSET_US + 900}, 'timesync_uncertainty_exceeded'),
    ({'timestamp': 1_000_000}, 'timesync_wrong_time_domain_or_stale'),
    ({'remote_timestamp': 1_000_000}, 'timesync_wrong_time_domain_or_stale'),
])
def test_invalid_timesync_fails_closed(changes, reason):
    gate = ready_gate()
    assert not gate.observe(status(EPOCH_US + 2000, **changes),
                            (EPOCH_US + 2000) * 1000, 2_000_000)
    assert gate.reason == reason
    assert not gate.ready


def test_missing_timesync_and_timeout_require_full_reconvergence():
    gate = TimeSyncGate(min_samples=2)
    with pytest.raises(SampleRejected):
        gate.validate_sample(EPOCH_US * 1000, EPOCH_US * 1000, 0)
    gate = ready_gate()
    segment = gate.segment
    assert not gate.tick((EPOCH_US + 3_000_000) * 1000, 3_000_000_000)
    assert gate.reason == 'timesync_timeout'
    assert gate.segment > segment
    assert not gate.observe(status(EPOCH_US + 3_001_000),
                            (EPOCH_US + 3_001_000) * 1000, 3_001_000_000)


def test_host_clock_step_and_offset_jump_reset_segment():
    gate = ready_gate()
    gate.tick((EPOCH_US + 1_002_000) * 1000, 2_000_000)
    assert not gate.ready and gate.reason == 'host_clock_jump'
    gate = ready_gate()
    changed = OFFSET_US + 2000
    assert not gate.observe(status(EPOCH_US + 2000, estimated_offset=changed,
                                   observed_offset=changed),
                            (EPOCH_US + 2000) * 1000, 2_000_000)
    assert gate.reason == 'timesync_offset_jump'


def test_duplicate_timesync_and_inferred_px4_reboot_reset():
    gate = ready_gate()
    assert not gate.observe(status(EPOCH_US + 1000),
                            (EPOCH_US + 2000) * 1000, 2_000_000)
    assert gate.reason == 'timesync_nonmonotonic_or_px4_restart'
    gate = ready_gate()
    # A new PX4 boot moves reconstructed boot time backwards despite host time advancing.
    reboot_offset = OFFSET_US - 500_000
    assert not gate.observe(status(EPOCH_US + 2000, estimated_offset=reboot_offset,
                                   observed_offset=reboot_offset),
                            (EPOCH_US + 2000) * 1000, 2_000_000)
    assert gate.reason == 'timesync_nonmonotonic_or_px4_restart'


def test_boot_domain_imu_does_not_fall_back_to_receive_time():
    gate = ready_gate()
    with pytest.raises(SampleRejected, match='imu_wrong_time_domain'):
        gate.validate_sample(1_000_000, (EPOCH_US + 1000) * 1000, 1_000_000)
    assert not gate.ready


def test_uncertainty_limit_is_an_explicit_run_policy():
    gate = ready_gate(max_uncertainty_us=1500)
    assert gate.observe(status(EPOCH_US + 2000, round_trip_time=2400),
                        (EPOCH_US + 2000) * 1000, 2_000_000)
    assert gate.uncertainty_us == 1210
    assert not gate.observe(status(EPOCH_US + 3000, round_trip_time=3000),
                            (EPOCH_US + 3000) * 1000, 3_000_000)
    assert gate.reason == 'timesync_uncertainty_exceeded'


@pytest.mark.parametrize('limit', [0, -1, float('nan'), float('inf')])
def test_invalid_uncertainty_limits_are_rejected(limit):
    with pytest.raises(ValueError, match='finite positive limits'):
        TimeSyncGate(max_uncertainty_us=limit)


def test_integration_endpoint_becomes_midpoint_with_nanosecond_precision():
    result = ImuAssembler().add(sensor(gyro_integral_dt=5001, accelerometer_integral_dt=5001))
    assert len(result) == 1
    assert result[0].timestamp_ns == EPOCH_US * 1000 - 2_500_500
    assert result[0].angular_velocity == (1, -2, -3)
    assert result[0].linear_acceleration == (4, -5, 9.81)


def test_accel_relative_timestamp_is_interpolated_without_extrapolation():
    assembler = ImuAssembler()
    # Accel midpoint trails gyro midpoint by 1 ms, so future data is required.
    assert assembler.add(sensor(accelerometer_timestamp_relative=-1000,
                                accelerometer_m_s2=[0, 0, 0])) == []
    result = assembler.add(sensor(EPOCH_US + 5000, accelerometer_timestamp_relative=-1000,
                                   accelerometer_m_s2=[10, 20, 30]))
    assert len(result) == 1
    assert result[0].timestamp_ns == EPOCH_US * 1000 - 2_500_000
    assert result[0].linear_acceleration == pytest.approx((2, -4, -6))
    assert result[0].interpolation_span_ns == 5_000_000


@pytest.mark.parametrize('changes,reason', [
    ({'accelerometer_timestamp_relative': RELATIVE_TIMESTAMP_INVALID}, 'relative_timestamp_invalid'),
    ({'gyro_clipping': 1}, 'imu_clipping'),
    ({'accelerometer_clipping': 1}, 'imu_clipping'),
    ({'gyro_integral_dt': 0}, 'invalid_integration_interval'),
    ({'accelerometer_integral_dt': 0}, 'invalid_integration_interval'),
    ({'gyro_integral_dt': 200000}, 'integration_interval_too_long'),
    ({'accelerometer_timestamp_relative': -30000}, 'sensor_skew_exceeded'),
    ({'accelerometer_m_s2': [1, 2, float('inf')]}, 'nonfinite'),
])
def test_unusable_measurements_rejected(changes, reason):
    with pytest.raises(SampleRejected, match=reason):
        ImuAssembler().add(sensor(**changes))


def test_sample_gap_nonmonotonic_and_calibration_change_break_continuity():
    for second, reason in [
        (sensor(EPOCH_US + 101000), 'imu_sample_gap'),
        (sensor(EPOCH_US), 'imu_timestamp_nonmonotonic'),
        (sensor(EPOCH_US + 5000, gyro_calibration_count=1), 'imu_calibration_changed'),
    ]:
        assembler = ImuAssembler()
        assembler.add(sensor())
        with pytest.raises(SampleRejected, match=reason):
            assembler.add(second)
        assert assembler.last_input_ns is None


def test_interpolation_never_crosses_reset_or_large_gap():
    assembler = ImuAssembler(interpolation_max_gap_s=0.01)
    assembler.add(sensor(accelerometer_timestamp_relative=-1000))
    assembler.reset()
    assert assembler.add(sensor(EPOCH_US + 5000, accelerometer_timestamp_relative=-1000)) == []
    assert len(assembler.gyros) == 1
    # Long accel brackets cause rejection, not extrapolation across missing data.
    result = assembler.add(sensor(EPOCH_US + 20000, accelerometer_timestamp_relative=-1000))
    assert result == []
    assert assembler.dropped_unbracketed == 1


def test_accel_ahead_discards_only_unbracketed_first_gyro():
    assembler = ImuAssembler()
    assert assembler.add(sensor(accelerometer_timestamp_relative=1000)) == []
    result = assembler.add(sensor(EPOCH_US + 5000, accelerometer_timestamp_relative=1000))
    assert len(result) == 1
    assert assembler.dropped_unbracketed == 1
