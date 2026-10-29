"""Single entrypoint for real bench capture. No flight control commands."""
import argparse
import csv
import os
from pathlib import Path
import shutil
import signal
import sqlite3
import subprocess
import sys
import time

import yaml

from .config import default_rig, load_rig, camera_parameters, capture_parameters, imu_provenance
from .ownership import DeviceLock, verify_usb_camera, verify_serial_free
from .preflight import SensorProbe
from .mavlink_contract import PROFILE as MAVLINK_PROFILE, mavlink_parameters, profile_annotations
from spr_diagnostics.run import Run, atomic_yaml, git_baseline, require_space, sha256


class CaptureCompleted(Exception):
    """A successful user-requested capture exit ends the supervised run."""


class Supervisor:
    def __init__(self, run):
        self.run = run
        self.children = []
        self.stopping = False
        self.optional_children = set()
        self.reported_optional_exits = set()

    def start(self, name, command, env, optional=False):
        log = (self.run.path / (name + ".log")).open("w")
        try:
            proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                    start_new_session=True, env=env)
        finally:
            log.close()
        self.children.append((name, proc))
        if optional:
            self.optional_children.add(name)
        self.run.event("process_started", component=name, pid=proc.pid, command=command)
        return proc

    def check(self, allow_capture_completion=False):
        for name, proc in self.children:
            if proc.poll() is not None:
                if name in self.optional_children:
                    if name not in self.reported_optional_exits:
                        self.run.event("optional_process_stopped", component=name, returncode=proc.returncode)
                        self.reported_optional_exits.add(name)
                    continue
                if name == "capture" and proc.returncode == 0 and allow_capture_completion:
                    raise CaptureCompleted()
                raise RuntimeError(f"{name} exited unexpectedly with code {proc.returncode}")

    def stop(self):
        self.stopping = True
        def send(proc, signum):
            try:
                os.killpg(proc.pid, signum)
            except ProcessLookupError:
                pass
        for name, proc in reversed(self.children):
            if proc.poll() is None:
                send(proc, signal.SIGINT)
                try:
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    send(proc, signal.SIGTERM)
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        send(proc, signal.SIGKILL)
                        proc.wait()
            self.run.event("process_stopped", component=name, returncode=proc.returncode)


def parameter_file(run, name, parameters):
    destination = run.path / (name + "_params.yaml")
    atomic_yaml(destination, {"/**": {"ros__parameters": parameters}})
    return str(destination)


def bag_evidence(path, topics):
    metadata_file = path / "metadata.yaml"
    if not metadata_file.exists():
        raise ValueError("bag was not finalized; metadata.yaml missing")
    metadata = yaml.safe_load(metadata_file.read_text())["rosbag2_bagfile_information"]
    counts = {}
    for file in path.glob("*.db3"):
        connection = sqlite3.connect(f"file:{file}?mode=ro", uri=True)
        try:
            for topic_id, name in connection.execute("SELECT id, name FROM topics"):
                count, minimum = connection.execute(
                    "SELECT COUNT(*), MIN(length(data)) FROM messages WHERE topic_id=?", (topic_id,)
                ).fetchone()
                if count and minimum <= 0:
                    raise ValueError(f"empty serialized payload on {name}")
                counts[name] = counts.get(name, 0) + count
        finally:
            connection.close()
    missing = [topic for topic in topics if counts.get(topic, 0) < 2]
    if missing:
        raise ValueError(f"bag missing or empty streams: {missing}")
    files = {}
    for file in sorted(path.rglob("*")):
        if file.is_symlink():
            raise ValueError(f"bag contains a symlink instead of immutable input: {file}")
        if file.is_file():
            files[file.relative_to(path).as_posix()] = sha256(file)
    return {"message_counts": counts, "metadata_sha256": sha256(metadata_file),
            "files_sha256": files,
            "duration_ns": metadata["duration"]["nanoseconds"]}


def record(args):
    rig = load_rig(args.rig or default_rig(args.profile))
    if rig.get("calibration_profile", "dds_runtime") != args.profile:
        raise ValueError("--profile disagrees with the selected rig; DDS and MAVLink cannot be mixed")
    if args.show_ui:
        rig["capture"]["show_ui"] = True
    baseline = git_baseline(args.repository)
    run = Run(args.runs, args.mode, rig, baseline)
    run.manifest.update(profile_annotations(args.profile), synthetic=False)
    print(f"run_dir={run.path}", flush=True)
    supervisor = Supervisor(run)
    locks = []
    probe = None
    ros_started = False
    interrupted = False
    failure = None
    quality_errors = []
    evidence = {}
    camera = args.mode not in ("allan", "imu")
    imu = args.mode in ("joint", "allan", "imu")
    mavlink = args.profile == MAVLINK_PROFILE
    capture_boards = args.mode == "stereo" or (args.mode == "joint" and args.save_board_images)
    env = dict(os.environ, ROS_LOG_DIR=str(run.path / "ros"),
               OMP_NUM_THREADS="2", OPENBLAS_NUM_THREADS="1", OPENCV_FOR_THREADS_NUM="2")
    # Bounded live test; never silently leave detached processes behind.
    def stop_requested(signum, frame):
        nonlocal interrupted
        interrupted = True
    previous = {sig: signal.signal(sig, stop_requested) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        if not args.experimental and not baseline["effective_clean"]:
            raise RuntimeError("formal capture requires a reproducible committed Git baseline: " +
                               "; ".join(baseline["provenance_errors"]) + "; use --experimental for debugging")
        run.manifest["experimental"] = args.experimental
        require_space(run.path, int(rig["record"]["min_free_gb"] * 1e9))
        if camera:
            locks.append(DeviceLock("camera:" + rig["camera"]["serial_no"]).acquire())
            run.manifest["camera"] = dict(rig["camera"])
            run.manifest["camera"]["usb"] = verify_usb_camera(rig["camera"]["usb_serial_no"])
            camera_yaml = parameter_file(run, "camera", camera_parameters(rig))
            supervisor.start("camera", ["ros2", "run", "realsense2_camera",
                "realsense2_camera_node", "--ros-args", "-r", "__ns:=/",
                "-r", "__node:=camera", "--params-file", camera_yaml], env)
        if imu and mavlink:
            # The serial owner is this child, not the supervisor. Holding its
            # DeviceLock in both processes would prevent opening the endpoint.
            verify_serial_free(rig["mavlink"]["serial_port"])
            params = parameter_file(run, "mavlink_imu", mavlink_parameters(rig, run.path / "mavlink"))
            dependencies = os.environ.get("SPR_MAVLINK_PYTHONPATH", "")
            child_env = dict(env)
            if dependencies:
                child_env["PYTHONPATH"] = dependencies + os.pathsep + env.get("PYTHONPATH", "")
            supervisor.start("mavlink_imu", ["/usr/bin/python3", "-m", "drone_bringup.mavlink_imu",
                "--ros-args", "--params-file", params], child_env)
        elif imu:
            locks.append(DeviceLock("serial:" + str(Path(rig["agent"]["serial_port"]).resolve())).acquire())
            verify_serial_free(rig["agent"]["serial_port"])
            agent = shutil.which(os.path.expanduser(rig["agent"]["executable"]))
            if not agent:
                raise RuntimeError("MicroXRCEAgent not installed; see docs/environment.md")
            supervisor.start("agent", [agent, "serial", "--dev", rig["agent"]["serial_port"],
                                      "-b", str(rig["agent"]["baudrate"])], env)
            from ament_index_python.packages import get_package_share_directory
            bridge_config = str(Path(get_package_share_directory("px4_bridge")) / "config/px4.yaml")
            bridge_overrides = parameter_file(run, "bridge", {
                "imu_frame_id": rig["imu"]["frame_id"], "imu_topic": rig["imu"]["topic"],
                "min_rate_hz": rig["imu"]["min_rate_hz"], "max_gap_s": rig["imu"]["max_gap_s"],
                "input_time_domain": "agent_synced", "timestamp_source": "integration_midpoint"})
            supervisor.start("bridge", ["ros2", "run", "px4_bridge", "px4_bridge",
                "--ros-args", "--params-file", bridge_config, "--params-file", bridge_overrides], env)
        import rclpy
        rclpy.init()
        ros_started = True
        probe = SensorProbe(rig, camera, imu)
        beginning = time.monotonic()
        deadline = beginning + rig["record"]["startup_timeout_s"]
        last_error = "no data"
        while time.monotonic() < deadline and not interrupted:
            supervisor.check()
            probe.spin()
            if time.monotonic() - beginning < rig["record"]["preflight_s"]:
                continue
            try:
                evidence["preflight"] = probe.validate()
                break
            except ValueError as error:
                last_error = str(error)
        else:
            raise RuntimeError("preflight failed: " + last_error)
        run.event("preflight_passed", metrics=evidence["preflight"])
        if camera:
            snapshot = probe.camera_configuration()
            run.manifest["camera"]["observed_snapshot"] = snapshot
        if imu and mavlink:
            # Parameter-service RPCs can consume several seconds. Only a fresh
            # subsequent diagnostic can bind a valid affine fit to acquisition.
            evidence["preflight"] = probe.fresh_imu_summary()
        if imu:
            sync = evidence["preflight"]["imu_sync"]
            run.manifest["imu"] = imu_provenance(rig, baseline, sync)
        run.write()
        topics = []
        if camera:
            topics.extend(rig["camera"][k] for k in ("left_image_topic", "right_image_topic",
                                                      "left_info_topic", "right_info_topic"))
        if imu:
            topics.extend([rig["imu"]["topic"], "/spr/diagnostics"])
        qos = {topic: {"reliability": "best_effort" if topic == rig["imu"]["topic"] or
                       topic == "/spr/diagnostics" else "reliable",
                       "history": "keep_last", "depth": 100 if topic == rig["imu"]["topic"] else 5,
                       "durability": "volatile"} for topic in topics}
        qos_path = run.path / "recorder_qos.yaml"
        atomic_yaml(qos_path, qos)
        supervisor.start("recorder", ["ros2", "bag", "record", "--storage", rig["record"]["storage_id"],
            "--output", str(run.path / "bag"), "--qos-profile-overrides-path", str(qos_path),
            *topics], env)
        if capture_boards:
            capture_yaml = parameter_file(run, "capture", capture_parameters(rig, run.path / "images"))
            capture_exe = args.capture_executable
            cmd = [str(capture_exe)] if capture_exe else ["ros2", "run", "calibration", "capture"]
            supervisor.start("capture", [*cmd, "--ros-args", "--params-file", capture_yaml], env,
                             optional=args.mode == "joint")
        if args.mode == "allan":
            print("Stationary Allan recording: keep the complete camera/FC assembly motionless for at least one hour.", flush=True)
        elif args.mode == "joint":
            print("Joint recording: keep the board fixed; move the rigid camera/FC assembly to excite all axes.", flush=True)
        elif args.mode == "imu":
            print("MAVLink IMU bench test: recording measured rates and clock synchronization; this is not an Allan dataset.", flush=True)
        else:
            print("Live capture running; move the board through distances, angles and image corners.", flush=True)
        beginning = last_tick = time.monotonic()
        while time.monotonic() - beginning < args.duration and not interrupted:
            supervisor.check(allow_capture_completion=True)
            probe.spin()
            now = time.monotonic()
            if now - last_tick >= 1:
                free = require_space(run.path, int(rig["record"]["min_free_gb"] * 1e9))
                try:
                    summary = probe.validate()
                except ValueError as error:
                    if not args.experimental:
                        raise
                    summary = {"quality_error": str(error)}
                    quality_errors.append(str(error))
                    run.event("sensor_quality_failed", "ERROR", error=str(error))
                run.metric(uptime_s=round(now - beginning, 3), free_bytes=free,
                           sensor_summary=yaml.safe_dump(summary, default_flow_style=True).strip())
                last_tick = now
    except CaptureCompleted:
        run.event("capture_completed", component="capture", reason="successful capture exit")
    except Exception as error:
        failure = str(error)
        run.event("capture_failed", "ERROR", error=failure)
    finally:
        if failure is None and camera and probe:
            try:
                final_snapshot = probe.camera_configuration()
                run.manifest["camera"]["final_observed_snapshot"] = final_snapshot
                initial = run.manifest["camera"].get("observed_snapshot", {})
                if initial.get("parameters") != final_snapshot["parameters"]:
                    raise ValueError("camera parameters changed during the capture")
            except Exception as error:
                failure = str(error)
                run.event("camera_provenance_failed", "ERROR", error=failure)
        if failure is None and imu and mavlink and probe:
            try:
                evidence["final_preflight"] = probe.fresh_imu_summary()
                run.manifest["imu"]["final_timesync_snapshot"] = evidence["final_preflight"]["imu_sync"]
            except Exception as error:
                failure = str(error)
                run.event("imu_provenance_failed", "ERROR", error=failure)
        supervisor.stop()
        if probe:
            probe.close()
        if ros_started:
            rclpy.shutdown()
        for lock in reversed(locks):
            lock.close()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    if failure is None:
        try:
            evidence["bag"] = bag_evidence(run.path / "bag", topics)
            if capture_boards:
                counts = []
                for index in (run.path / "images").glob("*/index.csv"):
                    with index.open() as stream:
                        counts.append(sum(1 for _ in csv.DictReader(stream)))
                evidence["image_pairs"] = sum(counts)
                if args.mode == "stereo" and evidence["image_pairs"] < rig["capture"]["min_pairs"]:
                    raise ValueError(f"insufficient board pairs: {evidence['image_pairs']}")
                codes = {name: p.returncode for name, p in supervisor.children}
                if args.mode == "stereo" and codes.get("capture") != 0:
                    raise ValueError("capture quality gate failed; inspect capture.log and images/meta.yaml")
            if args.mode == "allan" and evidence["bag"]["duration_ns"] < 3600e9:
                raise ValueError("Allan dataset is shorter than one hour")
        except Exception as error:
            failure = str(error)
    if quality_errors:
        evidence["sensor_quality_errors"] = quality_errors
        failure = failure or "sensor quality gate failed during experimental capture"
    run.finish(failure is None, failure or "real sensor capture finalized", **evidence)
    print(f"status={run.manifest['status']} reason={run.manifest['reason']}", flush=True)
    return 1 if failure else 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    check = sub.add_parser("check-rig", help="validate rig schema without hardware")
    check.add_argument("--rig", type=Path)
    check.add_argument("--profile", choices=(MAVLINK_PROFILE, "dds_runtime"), default=MAVLINK_PROFILE)
    capture = sub.add_parser("record", help="start unique device processes and record real sensor data")
    capture.add_argument("--mode", choices=("camera", "stereo", "joint", "allan", "imu"), required=True)
    capture.add_argument("--profile", choices=(MAVLINK_PROFILE, "dds_runtime"), default=MAVLINK_PROFILE)
    capture.add_argument("--rig", type=Path)
    capture.add_argument("--repository", type=Path, default=Path.cwd())
    capture.add_argument("--runs", type=Path, default=Path.cwd() / "runs")
    capture.add_argument("--duration", type=float, default=120.0)
    capture.add_argument("--experimental", action="store_true")
    capture.add_argument("--show-ui", action="store_true", help="show calibration preview windows")
    capture.add_argument("--save-board-images", action="store_true",
                         help="optionally save checkerboard PNGs alongside a joint bag; no stereo coverage gate")
    capture.add_argument("--capture-executable", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.command == "check-rig":
        rig = load_rig(args.rig or default_rig(args.profile))
        if rig.get("calibration_profile", "dds_runtime") != args.profile:
            parser.error("--profile disagrees with the selected rig")
        print("rig schema OK")
        return 0
    if not (args.duration > 0 and args.duration < 86400):
        parser.error("--duration must be between zero and 86400 seconds")
    if args.mode == "imu" and (args.profile != MAVLINK_PROFILE or args.duration < 10):
        parser.error("--mode imu requires --profile mavlink_calibration and --duration >=10")
    return record(args)


if __name__ == "__main__":
    sys.exit(main())
