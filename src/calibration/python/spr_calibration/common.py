"""Portable artifact formats and explicit calibration conventions."""

from datetime import datetime, timezone
import hashlib
import html
import json
from importlib import metadata
import os
from pathlib import Path
import platform
import subprocess
import tempfile

import numpy as np
import yaml


class CalibrationError(ValueError):
    """A calibration contract or quality gate failed."""


def read_yaml(path):
    text = Path(path).read_text(encoding="utf-8")
    if text.startswith("%YAML:1.0"):
        raise CalibrationError(f"{path}: OpenCV YAML is allowed only in final OpenVINS exports")
    result = yaml.safe_load(text)
    if not isinstance(result, dict):
        raise CalibrationError(f"{path}: expected a YAML mapping")
    return result


def write_yaml(path, data):
    path = Path(path)
    fd, temporary = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def hash_input(path):
    """Hash file bytes and relative names; do not follow directory symlinks."""
    path = Path(path)
    if path.is_file():
        return sha256_file(path)
    if not path.is_dir():
        raise CalibrationError(f"Input does not exist: {path}")
    entries = []
    for child in sorted(path.rglob("*")):
        if child.is_symlink():
            raise CalibrationError(f"Symlink is not an immutable input: {child}")
        if child.is_file():
            entries.append([child.relative_to(path).as_posix(), sha256_file(child)])
    if not entries:
        raise CalibrationError(f"Empty input directory: {path}")
    return hashlib.sha256(json.dumps(entries, separators=(",", ":")).encode()).hexdigest()


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def git_revision():
    result = subprocess.run(["git", "rev-parse", "HEAD"], text=True, capture_output=True)
    return result.stdout.strip() if result.returncode == 0 else "UNCOMMITTED"


def tool_versions():
    versions = {"python": platform.python_version(), "numpy": np.__version__, "PyYAML": yaml.__version__}
    for package in ("rosbags", "apsw", "lz4", "ruamel.yaml", "backports.zstd", "opencv-python"):
        try:
            versions[package] = metadata.version(package)
        except metadata.PackageNotFoundError:
            pass
    try:
        import cv2
        versions["opencv"] = cv2.__version__
    except ImportError:
        pass
    return versions


def new_output(path):
    path = Path(path).resolve()
    if "calibrations" in path.parts:
        raise CalibrationError("Intermediate output belongs in runs/, not calibrations/")
    path.mkdir(parents=True, exist_ok=False)
    return path


def report_html(path, title, result):
    """Single-file report; no network, external scripts, or hidden dependencies."""
    body = html.escape(yaml.safe_dump(result, sort_keys=False, allow_unicode=True))
    passed = result.get("status") == "PASSED"
    Path(path).write_text(
        "<!doctype html><html lang='en'><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<style>body{font:16px system-ui;max-width:1100px;margin:2rem auto;padding:1rem}"
        "pre{white-space:pre-wrap;background:#f1f4f7;padding:1rem}"
        ".state{color:" + ("#176533" if passed else "#a12424") + "}</style>"
        f"<title>{html.escape(title)}</title><h1>{html.escape(title)}</h1>"
        f"<p class='state'>{html.escape(str(result.get('status', 'UNVERIFIED')))}</p>"
        f"<pre>{body}</pre></html>", encoding="utf-8")


def finite_positive(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
        raise CalibrationError(f"{label} must be positive and finite")
    return float(value)


def transform(value, label):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise CalibrationError(f"{label} must be a finite 4x4 matrix")
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8):
        raise CalibrationError(f"{label} has an invalid homogeneous last row")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-5) or abs(np.linalg.det(rotation) - 1) > 1e-5:
        raise CalibrationError(f"{label} rotation is not in SO(3)")
    return matrix


def camera_chain(data, joint=False):
    if sorted(key for key in data if key.startswith("cam")) != ["cam0", "cam1"]:
        raise CalibrationError("Exactly cam0/cam1 are required")
    for name in ("cam0", "cam1"):
        camera = data[name]
        if camera.get("camera_model") != "pinhole" or camera.get("distortion_model") != "radtan":
            raise CalibrationError(f"{name}: pinhole + radtan required")
        intrinsic = np.asarray(camera.get("intrinsics", []), dtype=float)
        distortion = np.asarray(camera.get("distortion_coeffs", []), dtype=float)
        if intrinsic.shape != (4,) or not np.isfinite(intrinsic).all() or np.any(intrinsic[:2] <= 0):
            raise CalibrationError(f"{name}: intrinsics must be [fx,fy,cx,cy]")
        if distortion.shape != (4,) or not np.isfinite(distortion).all():
            raise CalibrationError(f"{name}: exactly four finite radtan coefficients required")
        resolution = camera.get("resolution", [])
        if len(resolution) != 2 or any(not isinstance(x, int) or isinstance(x, bool) or x <= 0 for x in resolution):
            raise CalibrationError(f"{name}: invalid resolution")
        if not isinstance(camera.get("rostopic"), str) or not camera["rostopic"].startswith("/"):
            raise CalibrationError(f"{name}: absolute rostopic required")
        if joint:
            transform(camera.get("T_cam_imu"), f"{name}.T_cam_imu")
            shift = camera.get("timeshift_cam_imu")
            if isinstance(shift, bool) or not isinstance(shift, (int, float)) or not np.isfinite(shift):
                raise CalibrationError(f"{name}: finite timeshift_cam_imu required")
    if data["cam0"]["rostopic"] == data["cam1"]["rostopic"]:
        raise CalibrationError("Left and right image topics must differ")
    stereo = transform(data["cam1"].get("T_cn_cnm1"), "cam1.T_cn_cnm1")
    if not 0.001 < np.linalg.norm(stereo[:3, 3]) < 1.0:
        raise CalibrationError("Stereo baseline must be between 1 mm and 1 m")
    if joint:
        expected = transform(data["cam1"]["T_cam_imu"], "cam1.T_cam_imu") @ np.linalg.inv(
            transform(data["cam0"]["T_cam_imu"], "cam0.T_cam_imu"))
        if not np.allclose(stereo, expected, atol=1e-5):
            raise CalibrationError("Joint T_cam_imu matrices disagree with stereo T_c1_c0")
    return data


NOISE_KEYS = ("accelerometer_noise_density", "accelerometer_random_walk",
              "gyroscope_noise_density", "gyroscope_random_walk", "update_rate")


def imu_config(data, profile="dds_runtime"):
    from .profiles import profile_config
    for key in NOISE_KEYS:
        finite_positive(data.get(key), key)
    contract = profile_config(profile)
    if data.get("rostopic") != contract["imu_topic"]:
        raise CalibrationError(f"{profile} IMU path must be {contract['imu_topic']} from {contract['source']}")
    return data


def target_config():
    """8 columns x 11 rows of INNER corners, 25 mm square edges."""
    return {"target_type": "checkerboard", "targetCols": 8, "targetRows": 11,
            "rowSpacingMeters": 0.025, "colSpacingMeters": 0.025}
