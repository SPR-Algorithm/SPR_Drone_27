"""Separate, opt-in MAVLink bench contract; frozen DDS settings are untouched."""
import ast
import copy
import math


PROFILE = "mavlink_calibration"
MIN_RATE_HZ = 140.0
SCOPE = "mavlink_140hz_test_only"


def profile_annotations(profile):
    result = {"calibration_profile": profile}
    if profile == PROFILE:
        result.update(calibration_phase="mavlink_140hz_test", deployment_scope=SCOPE,
                      minimum_observed_imu_rate_hz=MIN_RATE_HZ,
                      original_dds_200hz_requirement_satisfied=False)
    return result


def diagnostic_snapshot(status, received_at_ns):
    """ROS byte-valued levels and string KeyValues become typed audit evidence."""
    level = status.level
    if isinstance(level, (bytes, bytearray)):
        level = int.from_bytes(level, "little")
    result = {}
    for value in status.values:
        try:
            result[value.key] = ast.literal_eval(value.value)
        except (ValueError, SyntaxError):
            result[value.key] = value.value
    result.update(level=int(level), message=status.message, received_at_ns=received_at_ns)
    return result


def mavlink_parameters(rig, evidence_dir):
    settings = rig["mavlink"]
    return {"device": settings["serial_port"], "baud": settings["baudrate"],
            "requested_rate_hz": settings["requested_rate_hz"],
            "min_rate_hz": rig["imu"]["min_rate_hz"],
            "imu_topic": rig["imu"]["topic"], "frame_id": rig["imu"]["frame_id"],
            "evidence_dir": str(evidence_dir), "max_gap_s": rig["imu"]["max_gap_s"],
            "sync_min_samples": settings["sync_min_samples"],
            "sync_max_uncertainty_us": settings["sync_max_uncertainty_us"],
            "deployment_scope": SCOPE}


def mavlink_provenance(rig, baseline, synchronization):
    snapshot = copy.deepcopy(synchronization)
    if snapshot.get("level") != 0 or str(snapshot.get("sync_ready")).lower() != "true":
        raise ValueError("MAVLink provenance requires a healthy synchronized diagnostic")
    required = {"source": "mavlink_highres_imu", "timestamp_mode": "mavlink_boot_to_host",
                "sync_protocol": "mavlink_timesync", "deployment_scope": SCOPE,
                "timestamp_source": "vehicle_imu.timestamp_sample_no_midpoint_reconstruction",
                "raw_imu": False, "integration_dt_available": False,
                "bias_policy": "px4_estimator_bias_subtracted"}
    if any(snapshot.get(key) != value for key, value in required.items()):
        raise ValueError("MAVLink diagnostic does not prove the selected source/timing semantics")
    for key, maximum in (("sync_uncertainty_us", 1000), ("clock_fit_residual_us", 1000)):
        value = float(snapshot[key])
        if not math.isfinite(value) or not 0 <= value <= maximum:
            raise ValueError("MAVLink measured time uncertainty/residual exceeds 1 ms")
    rate = float(snapshot["rate_hz"])
    scale = float(snapshot["clock_scale"])
    offset = float(snapshot["host_minus_px4_offset_us"])
    if not all(math.isfinite(value) for value in (rate, scale, offset)) or scale <= 0 or rate < MIN_RATE_HZ:
        raise ValueError("MAVLink measured affine clock/rate does not pass the 140 Hz test contract")
    reference_fc, reference_host = snapshot["reference_px4_ns"], snapshot["reference_host_ns"]
    if any(isinstance(value, bool) or not isinstance(value, int) or value <= 0
           for value in (reference_fc, reference_host)):
        raise ValueError("MAVLink affine clock reference timestamps are missing")
    if abs((reference_host - reference_fc) / 1000 - offset) > 1:
        raise ValueError("MAVLink affine reference and local offset disagree")
    result = dict(rig["imu"], bridge_commit=baseline.get("commit"),
                  bridge_code_dirty=baseline.get("effective_clean") is not True,
                  approved_source_patches=copy.deepcopy(baseline.get("approved_patches", {})),
                  T_host_px4_offset_s=offset / 1e6, timesync_snapshot=snapshot,
                  timestamp_policy="affine boot-to-host mapping; transmitted sample endpoint; no invented midpoint")
    result.update({key: snapshot[key] for key in required})
    return result
