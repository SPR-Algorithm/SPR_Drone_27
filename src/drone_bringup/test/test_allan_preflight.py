"""Replay actual MAVLink cadence; never create or resample IMU measurements.

Fixture: samples 2245..3244 (one-based, excluding header), retaining only
stamp_ns/received_wall_ns from
runs/mavlink_ros_validation_140_20261029T131846Z/owner/samples.csv.
Source SHA-256: 28aefb62cdda4d70318005fc68bf5c19c1dfef9e7cb414c36e3f6ded80a881fe.
The contiguous excerpt spans more than five seconds so SensorProbe's bounded
window is exercised with actual source and receive timestamps.
"""
import csv
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

from drone_bringup.config import load_rig
from drone_bringup.preflight import SensorProbe, validate_streams


ROOT = Path(__file__).resolve().parents[1]
FIXTURE = Path(__file__).parent / "fixtures/mavlink_allan_timestamps_real_20261029.csv"


def measured_observations():
    with FIXTURE.open() as stream:
        return [{"stamp_ns": int(row["stamp_ns"]),
                 "received_ns": int(row["received_wall_ns"]),
                 "frame_id": "imu_link", "valid": True}
                for row in csv.DictReader(stream)]


@pytest.mark.parametrize("recent_only", [False, True])
def test_real_140hz_bench_cadence_fails_only_the_opted_in_allan_gate(recent_only):
    rig = load_rig(ROOT / "config/rig/d435i_mavlink.yaml")
    observations = measured_observations()
    assert len(observations) == 1000
    now_ns = observations[-1]["received_ns"]
    if recent_only:
        observations = [s for s in observations if s["received_ns"] >= now_ns - 5_000_000_000]
        assert 100 < len(observations) < 1000
    samples = {rig["imu"]["topic"]: observations}
    bench = validate_streams(samples, rig, now_ns, include_camera=False)
    assert bench[rig["imu"]["topic"]]["rate_hz"] >= 140
    assert bench[rig["imu"]["topic"]]["max_gap_s"] < .1
    with pytest.raises(ValueError, match="uniform sampling"):
        validate_streams(samples, rig, now_ns, include_camera=False,
                         allan_sampling=True)


def measured_probe(allan_sampling):
    """Exercise real validation without constructing ROS nodes or subscriptions."""
    probe = object.__new__(SensorProbe)
    probe.rig = load_rig(ROOT / "config/rig/d435i_mavlink.yaml")
    probe.camera = False
    probe.imu = True
    probe.allan_sampling = allan_sampling
    observations = measured_observations()
    now_ns = observations[-1]["received_ns"]
    probe.samples = {probe.rig["imu"]["topic"]: observations}
    probe.sync = {"level": 0, "received_at_ns": now_ns}
    probe.node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now_ns)),
        get_publishers_info_by_topic=lambda _topic: [
            SimpleNamespace(node_name="calibration_mavlink_imu")])
    return probe, now_ns


def test_probe_rechecks_the_allan_gate_on_its_recent_window():
    probe, _ = measured_probe(False)
    metrics = probe.validate()[probe.rig["imu"]["topic"]]
    assert 100 < metrics["count"] < 1000
    assert metrics["rate_hz"] >= 140
    probe.allan_sampling = True
    with pytest.raises(ValueError, match="uniform sampling"):
        probe.validate()


def test_fresh_imu_summary_keeps_the_allan_gate(monkeypatch):
    probe, now_ns = measured_probe(True)
    probe.spin = lambda _seconds: probe.sync.update(received_at_ns=now_ns + 1)
    ticks = iter([0., .01, 2.])
    monkeypatch.setattr("drone_bringup.preflight.time.monotonic", lambda: next(ticks))
    with pytest.raises(ValueError, match="uniform sampling"):
        probe.fresh_imu_summary(timeout_s=1.)


@pytest.mark.parametrize("mode,expected", [("allan", True), ("imu", False), ("joint", False)])
def test_cli_enables_allan_gate_only_for_mavlink_allan(monkeypatch, tmp_path, mode, expected):
    from drone_bringup import cli

    # Stop at probe construction; all process, device and ROS entry points below
    # are inert doubles, so this checks CLI routing without launching hardware.
    manifest = {}
    def finish(success, reason, **_evidence):
        manifest.update(status="PASSED" if success else "FAILED", reason=reason)

    run = SimpleNamespace(path=tmp_path, manifest=manifest, event=lambda *a, **k: None,
                          finish=finish)
    monkeypatch.setattr(cli, "Run", lambda *a: run)
    monkeypatch.setattr(cli, "Supervisor", lambda _run: SimpleNamespace(
        children=[], start=lambda *a, **k: None, stop=lambda: None))
    monkeypatch.setattr(cli, "git_baseline", lambda _path: {"effective_clean": True})
    monkeypatch.setattr(cli, "require_space", lambda *a: None)
    monkeypatch.setattr(cli, "verify_serial_free", lambda _port: None)
    monkeypatch.setattr(cli, "verify_usb_camera", lambda _serial: {})
    monkeypatch.setattr(cli, "parameter_file", lambda *a: "unused.yaml")
    lock = SimpleNamespace(close=lambda: None)
    lock.acquire = lambda: lock
    monkeypatch.setattr(cli, "DeviceLock", lambda _name: lock)
    monkeypatch.setattr(cli.signal, "signal", lambda *a: None)
    monkeypatch.setitem(sys.modules, "rclpy", SimpleNamespace(init=lambda: None,
                                                            shutdown=lambda: None))
    observed = []
    def inspect_probe(_rig, _camera, _imu, *, allan_sampling=False):
        observed.append(allan_sampling)
        raise ValueError("probe routing inspected")

    monkeypatch.setattr(cli, "SensorProbe", inspect_probe)
    args = SimpleNamespace(rig=ROOT / "config/rig/d435i_mavlink.yaml",
                           profile="mavlink_calibration", show_ui=False,
                           repository=ROOT, runs=tmp_path, mode=mode,
                           experimental=False, save_board_images=False)
    assert cli.record(args) == 1
    assert observed == [expected]
    assert manifest["reason"] == "probe routing inspected"
