"""Rig is the only source of device, topic, stream and board settings."""
import copy
import json
import math
from pathlib import Path

import jsonschema
import yaml


def default_rig(profile="mavlink_calibration"):
    from ament_index_python.packages import get_package_share_directory
    filename = "d435i_mavlink.yaml" if profile == "mavlink_calibration" else "d435i.yaml"
    return Path(get_package_share_directory("drone_bringup")) / "config/rig" / filename


def schema_path(rig_path, profile="dds_runtime"):
    # Installed and source-tree config use the same relative layout.
    here = Path(__file__).resolve()
    filename = "rig_mavlink.schema.json" if profile == "mavlink_calibration" else "rig.schema.json"
    source = here.parent.parent / "config" / filename
    if source.exists():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("drone_bringup")) / "config" / filename


def load_rig(path, schema=None):
    path = Path(path)
    value = yaml.safe_load(path.read_text())
    with Path(schema or schema_path(path, value.get("calibration_profile", "dds_runtime"))).open() as stream:
        jsonschema.Draft7Validator(json.load(stream)).validate(value)
    topics = [value["camera"][k] for k in
              ("left_image_topic", "right_image_topic", "left_info_topic", "right_info_topic")]
    if len(set(topics)) != 4:
        raise ValueError("camera topics must be distinct")
    # Namespace is an interface contract, not a launch substitution.
    expected = ["/camera/infra1/image_rect_raw", "/camera/infra2/image_rect_raw",
                "/camera/infra1/camera_info", "/camera/infra2/camera_info"]
    if topics != expected:
        raise ValueError("camera topic changes require an interface ADR and remapping update")
    return value


def camera_parameters(rig):
    cam = rig["camera"]
    return {
        "camera_name": "camera", "serial_no": cam["serial_no"],
        "enable_infra": False, "enable_infra1": True, "enable_infra2": True,
        "enable_depth": False, "enable_color": False,
        "enable_gyro": False, "enable_accel": False, "enable_motion": False,
        "depth_module.infra_profile": cam["profile"],
        "depth_module.infra1_format": "Y8", "depth_module.infra2_format": "Y8",
        "enable_sync": True, "depth_module.emitter_enabled": 0,
        "depth_module.enable_auto_exposure": cam["enable_auto_exposure"],
        "depth_module.exposure": cam["exposure"], "depth_module.gain": cam["gain"],
        "publish_tf": False,
        "infra1_qos": "DEFAULT", "infra2_qos": "DEFAULT",
        "infra1_info_qos": "DEFAULT", "infra2_info_qos": "DEFAULT",
    }


def capture_parameters(rig, output_dir):
    cam, board = rig["camera"], rig["board"]
    width, height, _ = map(int, cam["profile"].split("x"))
    params = copy.deepcopy(rig["capture"])
    params.update({k: cam[k] for k in ("left_image_topic", "right_image_topic",
                                      "left_info_topic", "right_info_topic")})
    params.update(board_cols=board["cols"], board_rows=board["rows"],
                  square_size=board["square_m"], output_dir=str(output_dir),
                  expected_width=width, expected_height=height,
                  max_stereo_delta_sec=cam["pair_tolerance_s"], opencv_threads=2)
    return params


def imu_provenance(rig, baseline, synchronization):
    """Snapshot measured DDS timing without inventing an uncommitted revision.

    PX4 already applies host synchronization before publishing SensorCombined.
    The host-minus-PX4 offset is recorded for audit, never added a second time.
    """
    if rig.get("calibration_profile") == "mavlink_calibration":
        from .mavlink_contract import mavlink_provenance
        return mavlink_provenance(rig, baseline, synchronization)
    snapshot = copy.deepcopy(synchronization)
    if snapshot.get("level") != 0 or str(snapshot.get("sync_ready")).lower() != "true":
        raise ValueError("IMU provenance requires a healthy synchronized bridge diagnostic")
    if snapshot.get("timestamp_mode") != "agent_synced_no_second_offset":
        raise ValueError("IMU diagnostic does not prove host timestamp conversion semantics")
    if snapshot.get("sync_protocol") != "uxrce_dds_timesync_status":
        raise ValueError("IMU diagnostic is not from DDS synchronization")
    offset = float(snapshot["host_minus_px4_offset_us"])
    uncertainty = float(snapshot["sync_uncertainty_us"])
    if not math.isfinite(offset) or not math.isfinite(uncertainty) or not 0 <= uncertainty <= 1000:
        raise ValueError("IMU measured time uncertainty must be finite and <=1 ms")
    return dict(rig["imu"], bridge_commit=baseline.get("commit"),
                bridge_code_dirty=baseline.get("effective_clean") is not True,
                approved_source_patches=copy.deepcopy(baseline.get("approved_patches", {})),
                T_host_px4_offset_s=offset / 1e6, timesync_snapshot=snapshot,
                timestamp_policy="host_ros_from_agent; no second offset application")
