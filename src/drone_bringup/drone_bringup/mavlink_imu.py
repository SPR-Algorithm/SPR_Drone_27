"""Bringup-owned, disarmed-only MAVLink IMU input for offline calibration.

PX4 v1.16 HIGHRES_IMU transmits vehicle_imu.timestamp_sample and bias-subtracted
integrated averages in FRD SI units. Integration periods and accelerometer sample
time are absent: preserve the transmitted timestamp; never invent a midpoint.
"""
from collections import Counter, deque
import argparse
import csv
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import statistics
import threading
import time

from .mavlink_time import ClockRejected, MavlinkClock
from .ownership import DeviceLock, verify_serial_free


DEFAULTS = {
    'device': '/dev/serial/by-id/usb-MicoAir_MicoAir743v2_0-if00',
    'baud': 115200, 'requested_rate_hz': 200.0, 'min_rate_hz': 200.0,
    'imu_topic': '/spr/calibration/imu', 'diagnostics_topic': '/spr/diagnostics',
    'frame_id': 'imu_link', 'evidence_dir': '', 'sync_min_samples': 20,
    'sync_max_uncertainty_us': 1000.0, 'max_gap_s': 0.1,
    'deployment_scope': 'mavlink_calibration_only',
}


class ArmedController(RuntimeError):
    pass


class MavlinkImuSession:
    """Own the serial endpoint and retain real wire/timing evidence.

    Use from one worker thread. on_sample receives a dictionary in FLU/host time.
    The parent must not hold the same DeviceLock while this owner runs.
    """
    def __init__(self, config, on_sample=lambda sample: None):
        self.config = dict(DEFAULTS, **config)
        c = self.config
        if not 1 <= c['requested_rate_hz'] <= 1000 or c['min_rate_hz'] <= 0:
            raise ValueError('MAVLink stream rates must be positive and <=1000 Hz')
        self.clock = MavlinkClock(c['sync_min_samples'], c['sync_max_uncertainty_us'])
        self.on_sample = on_sample
        self.link = None
        self.lock = None
        self.target = None
        self.armed = False
        self.last_heartbeat = None
        self.last_imu_mono = None
        self.last_output_mono = None
        self.last_boot_us = None
        self.last_mapped_ns = None
        self.pending_interval_at = None
        self.interval_requested = False
        self.interval_ack = None
        self.original_interval_us = None
        self.next_sync_ns = self.next_heartbeat_ns = 0
        self.next_status_ns = 0
        self.stats = deque(maxlen=2000)
        self.output_count = 0
        self.drops = Counter()
        self.failure = None
        self.latest_status = {}
        self.files = {}
        evidence = Path(c['evidence_dir']) if c['evidence_dir'] else None
        self.evidence = evidence
        if evidence is not None:
            evidence.mkdir(parents=True, exist_ok=True)
            for name in ['wire.jsonl', 'events.jsonl', 'sync.jsonl', 'samples.csv']:
                self.files[name] = (evidence / name).open('x')
            self.csv = csv.writer(self.files['samples.csv'])
            self.csv.writerow(['stamp_ns', 'px4_boot_us', 'received_wall_ns', 'received_monotonic_ns',
                               'gx', 'gy', 'gz', 'ax', 'ay', 'az', 'segment_id'])
        self.event('created', config=c, pymavlink_version=importlib.metadata.version('pymavlink'))

    def event(self, event, **fields):
        stream = self.files.get('events.jsonl')
        if stream:
            stream.write(json.dumps({'utc_ns': time.time_ns(), 'event': event, **fields}, default=str) + '\n')
            stream.flush()

    def open(self):
        from pymavlink import mavutil
        device = self.config['device']
        real = str(Path(device).resolve(strict=True))
        self.lock = DeviceLock('serial:' + real).acquire()
        try:
            verify_serial_free(device)
            self.link = mavutil.mavlink_connection(device, baud=self.config['baud'],
                autoreconnect=False, source_system=254, source_component=190)
            self.link.message_hooks.append(self._wire_message)
            self.target = None
            self.armed = False
            self.last_heartbeat = None
            self.last_boot_us = self.last_mapped_ns = None
            self.last_imu_mono = self.last_output_mono = None
            self.stats.clear()
            self.clock.reset('serial_connected_waiting_for_timesync')
            self.event('serial_opened', resolved_device=real)
            deadline = time.monotonic() + 4.0
            while self.target is None and time.monotonic() < deadline:
                self._heartbeat()
                message = self.link.recv_match(blocking=True, timeout=0.1)
                if message:
                    self._process(message, time.time_ns(), time.monotonic_ns())
            if self.target is None:
                raise TimeoutError('no disarmed PX4 autopilot heartbeat')
            self._command(510, 105, 0)  # Read original HIGHRES_IMU interval.
            self.pending_interval_at = time.monotonic_ns() + 200_000_000
        except Exception:
            self._close_transport(restore=False)
            raise

    def _wire_message(self, master, message):
        kind = message.get_type()
        if kind == 'BAD_DATA':
            self.drops['bad_mavlink_frame'] += 1
            self.event('bad_mavlink_frame', reason=getattr(message, 'reason', ''),
                       data_hex=bytes(getattr(message, 'data', b'')).hex())
            return
        stream = self.files.get('wire.jsonl')
        if stream:
            stream.write(json.dumps({'received_wall_ns': time.time_ns(),
                'received_monotonic_ns': time.monotonic_ns(),
                'system_id': message.get_srcSystem(), 'component_id': message.get_srcComponent(),
                'message': message.to_dict()}, default=lambda value: list(value)
                if isinstance(value, (bytes, bytearray)) else str(value)) + '\n')
        if (kind == 'HEARTBEAT' and message.autopilot == 12 and
                (self.target is None or (message.get_srcSystem(), message.get_srcComponent()) == self.target)):
            if message.base_mode & 128:
                self.armed = True
                raise ArmedController('armed FC heartbeat; calibration serial input stopped')

    def _command(self, command, message_id, interval):
        self.link.mav.command_long_send(*self.target, command, 0, message_id, interval, 0, 0, 0, 0, 0)
        self.event('transient_mavlink_command', command=command, message_id=message_id, interval_us=interval)

    def _heartbeat(self):
        if self.link is not None and time.monotonic_ns() >= self.next_heartbeat_ns:
            self.link.mav.heartbeat_send(0, 8, 0, 0, 0)  # Generic component; never an arming/control message.
            self.next_heartbeat_ns = time.monotonic_ns() + 1_000_000_000

    def poll(self, timeout_s=0.002):
        now_mono, now_wall = time.monotonic_ns(), time.time_ns()
        self._heartbeat()
        self.clock.tick(now_wall, now_mono)
        if self.last_heartbeat is not None and now_mono - self.last_heartbeat > 3_000_000_000:
            raise TimeoutError('PX4 heartbeat timeout')
        if self.pending_interval_at is not None and now_mono >= self.pending_interval_at:
            self._command(511, 105, round(1e6 / self.config['requested_rate_hz']))
            self.pending_interval_at = None
            self.interval_requested = True
        if self.target is not None and now_mono >= self.next_sync_ns:
            token = self.clock.request(now_wall, now_mono)
            self.link.mav.timesync_send(0, token)
            self.next_sync_ns = now_mono + 50_000_000
        message = self.link.recv_match(blocking=True, timeout=timeout_s)
        if message is not None:
            self._process(message, time.time_ns(), time.monotonic_ns())
        if now_mono >= self.next_status_ns:
            self.next_status_ns = now_mono + 1_000_000_000
            self.write_status()

    def _process(self, message, wall, mono):
        kind = message.get_type()
        origin = message.get_srcSystem(), message.get_srcComponent()
        if kind == 'HEARTBEAT' and message.autopilot == 12:
            if self.target is None:
                self.target = origin
                self.event('px4_selected', system_id=origin[0], component_id=origin[1])
            if origin == self.target:
                self.last_heartbeat = mono
            return
        if origin != self.target:
            return
        if kind == 'MESSAGE_INTERVAL' and message.message_id == 105 and not self.interval_requested:
            self.original_interval_us = message.interval_us
            self.event('original_stream_interval', interval_us=message.interval_us)
        elif kind == 'COMMAND_ACK' and message.command == 511:
            self.interval_ack = int(message.result)
            self.event('interval_acknowledged', result=self.interval_ack)
            if self.interval_ack != 0:
                raise RuntimeError('PX4 rejected HIGHRES_IMU stream interval')
        elif kind == 'TIMESYNC' and message.tc1 > 0:
            if self.clock.response(message.tc1, message.ts1, wall, mono):
                stream = self.files.get('sync.jsonl')
                if stream:
                    stream.write(json.dumps({'wall_ns': wall, 'monotonic_ns': mono,
                                              **self.clock.snapshot()}) + '\n')
        elif kind == 'HIGHRES_IMU':
            self._imu(message, wall, mono)

    def _imu(self, message, wall, mono):
        self.last_imu_mono = mono
        if message.fields_updated & 0x3f != 0x3f:
            self.drops['incomplete_accel_gyro_fields'] += 1
            return
        values = [message.xgyro, message.ygyro, message.zgyro,
                  message.xacc, message.yacc, message.zacc]
        if not all(math.isfinite(value) for value in values):
            self.drops['nonfinite_highres_imu'] += 1
            return
        boot = int(message.time_usec)
        if self.last_boot_us is not None:
            dt = (boot - self.last_boot_us) / 1e6
            if dt <= 0 or dt > self.config['max_gap_s']:
                self.clock.reset('imu_nonmonotonic_or_gap')
                self.last_mapped_ns = None
                self.stats.clear()
                self.event('imu_continuity_reset', dt_s=dt, segment_id=self.clock.segment)
        self.last_boot_us = boot
        self.stats.append((boot, mono))
        try:
            stamp = self.clock.map_boot_sample(boot, wall, mono)
            if self.last_mapped_ns is not None and stamp <= self.last_mapped_ns:
                self.clock.reset('mapped_imu_timestamp_nonmonotonic')
                self.last_mapped_ns = None
                raise ClockRejected('mapped_imu_timestamp_nonmonotonic')
        except ClockRejected as error:
            self.drops[str(error)] += 1
            return
        if self.interval_ack != 0:
            self.drops['waiting_for_interval_ack'] += 1
            return
        self.last_mapped_ns = stamp
        sample = {'stamp_ns': stamp, 'px4_boot_us': boot, 'received_wall_ns': wall,
                  'received_monotonic_ns': mono, 'segment_id': self.clock.segment,
                  'gyro': (values[0], -values[1], -values[2]),
                  'accel': (values[3], -values[4], -values[5])}
        self.output_count += 1
        self.last_output_mono = mono
        if 'samples.csv' in self.files:
            self.csv.writerow([stamp, boot, wall, mono, *sample['gyro'], *sample['accel'], self.clock.segment])
        self.on_sample(sample)

    def snapshot(self):
        now = time.monotonic_ns()
        rows = [row for row in self.stats if now - row[1] <= 5_000_000_000]
        rate = rx_rate = jitter = gap = 0.0
        if len(rows) >= 2:
            periods = [(b[0] - a[0]) / 1e6 for a, b in zip(rows, rows[1:])]
            if all(value > 0 for value in periods):
                rate = (len(rows) - 1) / ((rows[-1][0] - rows[0][0]) / 1e6)
                rx_rate = (len(rows) - 1) / ((rows[-1][1] - rows[0][1]) / 1e9)
                jitter = statistics.pstdev(periods) * 1e6
                gap = max(periods)
        status = self.clock.snapshot()
        healthy_input = self.last_imu_mono is not None and now - self.last_imu_mono <= round(self.config['max_gap_s'] * 1e9)
        healthy_output = self.last_output_mono is not None and now - self.last_output_mono <= round(self.config['max_gap_s'] * 1e9)
        healthy_heartbeat = self.last_heartbeat is not None and now - self.last_heartbeat < 3_000_000_000
        if self.failure or self.armed or not self.clock.ready or not healthy_input or not healthy_output or not healthy_heartbeat:
            level = 2
            message = self.failure or (self.clock.reason if not self.clock.ready else 'imu_or_heartbeat_stale')
        elif rate < self.config['min_rate_hz']:
            level, message = 1, 'mavlink_imu_rate_below_configured_minimum'
        else:
            level, message = 0, 'synchronized_mavlink_calibration_input'
        status.update(level=level, message=message, source='mavlink_highres_imu',
            topic=self.config['imu_topic'], frame_id=self.config['frame_id'], time_domain='host_ros',
            timestamp_source='vehicle_imu.timestamp_sample_no_midpoint_reconstruction',
            raw_imu=False, integration_dt_available=False,
            bias_policy='px4_estimator_bias_subtracted', orientation_available=False,
            covariance_known=False, frame_conversion='FRD_to_FLU_diag_1_minus1_minus1',
            deployment_scope=self.config['deployment_scope'], requested_rate_hz=self.config['requested_rate_hz'],
            min_rate_hz=self.config['min_rate_hz'], rate_hz=rate, rate_rx_hz=rx_rate,
            period_jitter_us=jitter, max_gap_s=gap, output_count=self.output_count,
            interval_ack=self.interval_ack, original_interval_us=self.original_interval_us,
            selected_autopilot=self.target, heartbeat_disarmed=healthy_heartbeat and not self.armed,
            drops=dict(self.drops))
        return status

    def write_status(self):
        self.latest_status = self.snapshot()
        if self.evidence is not None:
            for stream in self.files.values():
                stream.flush()
            (self.evidence / 'status.json').write_text(json.dumps(self.latest_status, indent=2) + '\n')

    def _close_transport(self, restore=True):
        if self.link is not None:
            if restore and self.interval_requested and not self.armed and self.target is not None:
                try:
                    original = self.original_interval_us if self.original_interval_us is not None else 0
                    self._command(511, 105, original)
                    acknowledged = False
                    deadline = time.monotonic() + 1.0
                    while time.monotonic() < deadline:
                        message = self.link.recv_match(blocking=True, timeout=0.05)
                        if (message is not None and message.get_type() == 'COMMAND_ACK' and
                                (message.get_srcSystem(), message.get_srcComponent()) == self.target and
                                message.command == 511):
                            acknowledged = message.result == 0
                            break
                    self._command(510, 105, 0)
                    readback = None
                    deadline = time.monotonic() + 1.0
                    while time.monotonic() < deadline:
                        message = self.link.recv_match(blocking=True, timeout=0.05)
                        if (message is not None and message.get_type() == 'MESSAGE_INTERVAL' and
                                (message.get_srcSystem(), message.get_srcComponent()) == self.target and
                                message.message_id == 105):
                            readback = message.interval_us
                            break
                    self.event('stream_interval_restored', requested_us=original,
                               acknowledged=acknowledged, readback_us=readback)
                    if not acknowledged or readback is None or (original > 0 and readback != original):
                        self.event('stream_restore_failed', error='restore acknowledgement/readback mismatch')
                    # Only restores the transient per-link stream interval; no PX4 parameter is written.
                except Exception as error:
                    self.event('stream_restore_failed', error=str(error))
            self.link.close()
            self.link = None
        if self.lock is not None:
            self.lock.close()
            self.lock = None
        self.interval_requested = False
        self.interval_ack = None
        self.original_interval_us = None

    def close(self):
        self.write_status()
        self._close_transport()
        self.event('serial_released')
        for stream in self.files.values():
            stream.close()
        if self.evidence is not None:
            files = [p for p in self.evidence.iterdir() if p.is_file() and p.name != 'sha256.json']
            (self.evidence / 'sha256.json').write_text(json.dumps({p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                                                 for p in files}, indent=2) + '\n')


def probe(seconds, evidence_dir, **config):
    """Run the same real owner without ROS; no synthetic inputs or calibration."""
    session = MavlinkImuSession(dict(config, evidence_dir=evidence_dir))
    try:
        session.open()
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            session.poll()
        status = session.snapshot()
        print(json.dumps(status, indent=2), flush=True)
        return status
    except Exception as error:
        session.failure = str(error)
        raise
    finally:
        session.close()


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rcl_interfaces.msg import SetParametersResult

    class CalibrationMavlinkImu(Node):
        def __init__(self):
            super().__init__('calibration_mavlink_imu')
            for name, value in DEFAULTS.items():
                self.declare_parameter(name, value)
            if self.get_parameter('use_sim_time').value:
                raise ValueError('real MAVLink calibration requires use_sim_time=false')
            config = {name: self.get_parameter(name).value for name in DEFAULTS}
            self.imu_publisher = self.create_publisher(Imu, config['imu_topic'],
                QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.diagnostic_publisher = self.create_publisher(DiagnosticArray, config['diagnostics_topic'],
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.session = MavlinkImuSession(config, self.publish_sample)
            self.add_on_set_parameters_callback(lambda parameters: SetParametersResult(
                successful=False, reason='MAVLink calibration parameters are fixed for a continuous run'))
            self.stop = threading.Event()
            self.worker = threading.Thread(target=self.read_serial, name='calibration_mavlink_serial', daemon=True)
            self.worker.start()
            self.create_timer(1.0, self.diagnostic)

        def publish_sample(self, sample):
            if self.stop.is_set() or not rclpy.ok():
                return
            output = Imu()
            output.header.frame_id = self.session.config['frame_id']
            output.header.stamp.sec = sample['stamp_ns'] // 1_000_000_000
            output.header.stamp.nanosec = sample['stamp_ns'] % 1_000_000_000
            output.orientation.w = 1.0
            output.orientation_covariance[0] = -1.0
            output.angular_velocity.x, output.angular_velocity.y, output.angular_velocity.z = sample['gyro']
            output.linear_acceleration.x, output.linear_acceleration.y, output.linear_acceleration.z = sample['accel']
            # ROS Imu convention: all-zero gyro/accel covariance means unknown.
            self.imu_publisher.publish(output)

        def read_serial(self):
            while not self.stop.is_set():
                try:
                    if self.session.link is None:
                        self.session.open()
                    self.session.poll()
                except ArmedController as error:
                    self.session.failure = str(error)
                    self.session.event('armed_abort', error=str(error))
                    self.session._close_transport(restore=False)
                    self.session.clock.reset('armed_controller_abort')
                    self.session.write_status()
                    self.stop.set()
                except (OSError, TimeoutError) as error:
                    self.session.clock.reset('serial_disconnected')
                    self.session.event('serial_reconnect_wait', error=str(error))
                    self.session._close_transport(restore=False)
                    self.session.write_status()
                    self.stop.wait(1.0)
                except Exception as error:
                    self.session.failure = str(error)
                    self.session.event('fatal_serial_error', error=str(error))
                    self.session.write_status()
                    self.stop.set()

        def diagnostic(self):
            # The wire worker atomically replaces this immutable snapshot. Do
            # not iterate its live deques from the ROS executor thread.
            status = dict(self.session.latest_status) or {'level': 2, 'message': 'waiting_for_serial_worker'}
            levels = [DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR]
            entry = DiagnosticStatus(name='drone_bringup/mavlink_imu', hardware_id=self.session.config['device'],
                                     level=levels[status['level']], message=status['message'])
            entry.values = [KeyValue(key=str(key), value=str(value)) for key, value in status.items()]
            output = DiagnosticArray()
            output.header.stamp = self.get_clock().now().to_msg()
            output.status = [entry]
            self.diagnostic_publisher.publish(output)

        def destroy_node(self):
            self.stop.set()
            self.worker.join(timeout=5)
            if self.worker.is_alive():
                raise RuntimeError('MAVLink worker did not stop; refusing to release owned resources')
            self.session.close()
            return super().destroy_node()

    rclpy.init(args=args)
    node = None
    try:
        node = CalibrationMavlinkImu()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--probe-seconds', type=float)
    parser.add_argument('--evidence-dir')
    parser.add_argument('--requested-rate-hz', type=float, default=200.0)
    parser.add_argument('--min-rate-hz', type=float, default=200.0)
    cli, remaining = parser.parse_known_args()
    if cli.probe_seconds:
        if not cli.evidence_dir:
            parser.error('--probe-seconds requires --evidence-dir')
        probe(cli.probe_seconds, cli.evidence_dir, requested_rate_hz=cli.requested_rate_hz,
              min_rate_hz=cli.min_rate_hz)
    else:
        main(remaining)
