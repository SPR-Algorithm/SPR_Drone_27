"""Explicit IMU acquisition contracts; calibration profiles never alias DDS."""

from .common import CalibrationError


DEFAULT_PROFILE = "dds_runtime"
MAVLINK_PROFILE = "mavlink_calibration"
MAVLINK_MIN_RATE_HZ = 140.0
MAVLINK_PHASE = "mavlink_140hz_test"
MAVLINK_SCOPE = "mavlink_140hz_test_only"
PROFILES = {
    DEFAULT_PROFILE: {
        "imu_topic": "/spr/imu", "source": "px4_bridge",
        "timestamp_mode": "agent_synced_no_second_offset",
        "sync_protocol": "uxrce_dds_timesync_status",
    },
    MAVLINK_PROFILE: {
        "imu_topic": "/spr/calibration/imu", "source": "mavlink_highres_imu",
        "timestamp_mode": "mavlink_boot_to_host",
        "sync_protocol": "mavlink_timesync",
        "min_rate_hz": MAVLINK_MIN_RATE_HZ,
        "calibration_phase": MAVLINK_PHASE,
        "deployment_scope": MAVLINK_SCOPE,
    },
}


def profile_config(profile=DEFAULT_PROFILE):
    if profile not in PROFILES:
        raise CalibrationError(f"Unknown calibration profile: {profile}")
    return PROFILES[profile]


def require_candidate_profile(manifest, requested):
    """Old candidates retain DDS semantics; MAVLink selection is always explicit."""
    profile_config(requested)
    recorded = manifest.get("calibration_profile", DEFAULT_PROFILE)
    if recorded != requested:
        raise CalibrationError(f"Candidate profile is {recorded}; select --profile {recorded}")
    if requested == MAVLINK_PROFILE:
        for key, expected in profile_annotations(requested).items():
            if manifest.get(key) != expected:
                raise CalibrationError(f"MAVLink test artifact must record {key}: {expected}")
    return requested


def profile_annotations(profile):
    """Carry the user's explicit 140 Hz test exception into each MAVLink artifact."""
    contract = profile_config(profile)
    result = {"calibration_profile": profile}
    if profile == MAVLINK_PROFILE:
        result.update(calibration_phase=contract["calibration_phase"],
                      deployment_scope=contract["deployment_scope"],
                      minimum_observed_imu_rate_hz=contract["min_rate_hz"],
                      original_dds_200hz_requirement_satisfied=False)
    return result
