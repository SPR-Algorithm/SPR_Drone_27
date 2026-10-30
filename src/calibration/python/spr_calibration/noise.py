"""Explicit, source-attributed engineering noise priors for the MAVLink test phase."""

from .common import CalibrationError, imu_config
from .profiles import MAVLINK_PROFILE


NOISE_PRIOR_UNITS = {
    "accelerometer_noise_density": "m/s^2/sqrt(Hz)",
    "accelerometer_random_walk": "m/s^3/sqrt(Hz)",
    "gyroscope_noise_density": "rad/s/sqrt(Hz)",
    "gyroscope_random_walk": "rad/s^2/sqrt(Hz)",
}


def validate_noise_prior(document, imu, profile):
    """Validate supplied values and attribution without estimating or filling values."""
    if profile != MAVLINK_PROFILE:
        raise CalibrationError("Engineering noise priors are allowed only for mavlink_calibration")
    if not isinstance(document, dict):
        raise CalibrationError("Noise prior must contain a YAML mapping")
    if type(document.get("schema_version")) is not int or document["schema_version"] != 1:
        raise CalibrationError("Noise prior schema_version must be 1")
    if document.get("calibration_profile") != profile or document.get("basis") != "engineering_prior":
        raise CalibrationError("Noise prior must explicitly select mavlink_calibration and basis engineering_prior")
    for key in ("noise_model_measured", "original_allan_requirement_satisfied"):
        if document.get(key) is not False:
            raise CalibrationError(f"Engineering noise prior must record {key}: false")
    if not isinstance(imu, dict) or not isinstance(document.get("imu"), dict):
        raise CalibrationError("Noise prior and supplied IMU configuration must contain mappings")
    supplied_imu = imu_config(imu, profile)
    prior_imu = imu_config(document["imu"], profile)
    if prior_imu != supplied_imu:
        raise CalibrationError("Noise prior imu must exactly match the supplied IMU configuration")
    sources = document.get("parameter_sources")
    if not isinstance(sources, dict):
        raise CalibrationError("Noise prior parameter_sources must attribute all four noise coefficients")
    for key, unit in NOISE_PRIOR_UNITS.items():
        source = sources.get(key)
        if not isinstance(source, dict):
            raise CalibrationError(f"Noise prior parameter_sources.{key} is missing")
        if source.get("kind") not in ("datasheet_prior", "engineering_assumption"):
            raise CalibrationError(f"Noise prior {key}.kind must be datasheet_prior or engineering_assumption")
        for field in ("reference", "rationale", "unit"):
            if not isinstance(source.get(field), str) or not source[field].strip():
                raise CalibrationError(f"Noise prior {key}.{field} must be a nonempty string")
        if source["unit"] != unit:
            raise CalibrationError(f"Noise prior {key}.unit must be {unit}")
    return {"noise_basis": "engineering_prior", "noise_model_measured": False,
            "original_allan_requirement_satisfied": False}
