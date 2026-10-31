"""Pinned, network-isolated real Kalibr invocation; never synthesize results."""

import json
import os
from pathlib import Path
import re
import shutil
import subprocess
from time import monotonic

import numpy as np

from .bags import convert_bag
from .common import (CalibrationError, camera_chain, hash_input,
                     imu_config, new_output, read_yaml, report_html, sha256_file,
                     target_config, tool_versions, utc_now, write_yaml)
from .profiles import DEFAULT_PROFILE, MAVLINK_PROFILE, profile_annotations, profile_config
from .noise import validate_noise_prior


DEFAULT_KALIBR_IMAGE = "spr-kalibr-cholmod:local"


def stream_kalibr_output(lines, log):
    """Keep the full log; show progress at 1 Hz plus stage/end snapshots."""
    last_progress_time = None
    pending_progress = None
    try:
        for line in lines:
            log.write(line)
            log.flush()
            if line.lstrip().startswith("Progress "):
                now = monotonic()
                if last_progress_time is None or now - last_progress_time >= 1.0:
                    print(line, end="", flush=True)
                    last_progress_time = now
                    pending_progress = None
                else:
                    pending_progress = line
            else:
                if pending_progress is not None:
                    print(pending_progress, end="", flush=True)
                    pending_progress = None
                print(line, end="", flush=True)
                last_progress_time = None
    finally:
        if pending_progress is not None:
            print(pending_progress, end="", flush=True)


def provenance_errors(manifest, camera_required=True, profile=DEFAULT_PROFILE):
    errors = []
    contract = profile_config(profile)
    imu, camera = manifest.get("imu", {}), manifest.get("camera", {})
    if manifest.get("calibration_profile", DEFAULT_PROFILE) != profile:
        errors.append("Acquisition calibration_profile differs from the explicitly selected profile")
    for key, expected in (("source", contract["source"]), ("frame_id", "imu_link"), ("time_domain", "host_ros")):
        if imu.get(key) != expected:
            errors.append(f"imu.{key} must be {expected}")
    if profile == MAVLINK_PROFILE and imu.get("topic") != contract["imu_topic"]:
        errors.append(f"imu.topic must be {contract['imu_topic']}")
    if not imu.get("bridge_commit") or imu.get("bridge_commit") in ("unknown", "UNCOMMITTED"):
        errors.append("immutable imu.bridge_commit is missing")
    elif not re.fullmatch(r"(?:[0-9a-f]{7,40}|[^\s]+-[0-9]+-g[0-9a-f]{7,40})", str(imu["bridge_commit"])):
        errors.append("imu.bridge_commit must identify an immutable git revision")
    offset = imu.get("T_host_px4_offset_s")
    if isinstance(offset, bool) or not isinstance(offset, (float, int)) or not np.isfinite(offset):
        errors.append("measured imu.T_host_px4_offset_s is missing")
    if not imu.get("timesync_snapshot"):
        errors.append("imu.timesync_snapshot (measured bridge diagnostic) is missing")
    else:
        snapshot = imu["timesync_snapshot"]
        if not isinstance(snapshot, dict):
            errors.append("imu.timesync_snapshot must contain the measured diagnostic fields")
        else:
            if snapshot.get("level") != 0 or str(snapshot.get("sync_ready")).lower() != "true":
                errors.append("IMU snapshot must prove a healthy synchronized bridge")
            if snapshot.get("timestamp_mode") != contract["timestamp_mode"] or snapshot.get("sync_protocol") != contract["sync_protocol"]:
                errors.append(f"IMU snapshot does not prove {profile} host-time conversion semantics")
            try:
                uncertainty = float(snapshot["sync_uncertainty_us"])
                snapshot_offset = float(snapshot["host_minus_px4_offset_us"]) / 1e6
                budget = 1000.0
                if profile == DEFAULT_PROFILE:
                    # Older DDS recordings predate the explicit run budget.
                    recorded_budget = snapshot.get("sync_budget_us", 1000.0)
                    if isinstance(recorded_budget, bool):
                        raise ValueError("a boolean is not a synchronization budget")
                    budget = float(recorded_budget)
                if (not np.isfinite(budget) or budget <= 0 or not np.isfinite(uncertainty)
                        or not 0 <= uncertainty <= budget):
                    errors.append("Measured IMU time uncertainty must be finite, nonnegative and within a finite positive sync budget")
                if not np.isfinite(snapshot_offset) or not isinstance(offset, (float, int)) or abs(offset - snapshot_offset) > 1e-9:
                    errors.append("Recorded T_host_px4 offset disagrees with the diagnostic snapshot")
            except (KeyError, TypeError, ValueError):
                errors.append("Measured IMU timing uncertainty/offset/budget fields are missing or invalid")
    if imu.get("bridge_code_dirty") is not False:
        errors.append("IMU bridge must have a recorded clean Git baseline")
    if profile == MAVLINK_PROFILE:
        # A requested MAVLink interval is not evidence of an achieved sample rate.
        # Metadata must come from the actual HIGHRES_IMU receive/publish monitor.
        snapshot = imu.get("timesync_snapshot", {})
        if not isinstance(snapshot, dict):
            snapshot = {}
        try:
            scale = float(snapshot["clock_scale"])
            residual = float(snapshot["clock_fit_residual_us"])
            reference_fc = snapshot["reference_px4_ns"]
            reference_host = snapshot["reference_host_ns"]
            if not np.isfinite(scale) or scale <= 0 or not np.isfinite(residual) or not 0 <= residual <= 1000:
                errors.append("Measured MAVLink affine clock fit is invalid or exceeds 1 ms residual")
            if any(isinstance(value, bool) or not isinstance(value, int) or value <= 0
                   for value in (reference_fc, reference_host)):
                errors.append("Measured MAVLink affine clock reference timestamps are invalid")
            elif abs((reference_host - reference_fc) / 1e3 - float(snapshot["host_minus_px4_offset_us"])) > 1.0:
                errors.append("MAVLink local time offset disagrees with its affine reference timestamps")
        except (KeyError, TypeError, ValueError):
            errors.append("Measured MAVLink affine clock scale/reference/residual evidence is missing")
        try:
            rate = float(snapshot["rate_hz"])
            if not np.isfinite(rate) or rate < contract["min_rate_hz"]:
                errors.append(f"MAVLink HIGHRES_IMU observed rate must be >={contract['min_rate_hz']:g} Hz for this test phase")
        except (KeyError, TypeError, ValueError):
            errors.append("Measured MAVLink HIGHRES_IMU rate_hz is missing")
        semantics = {"timestamp_source": "vehicle_imu.timestamp_sample_no_midpoint_reconstruction",
                     "raw_imu": False, "integration_dt_available": False,
                     "bias_policy": "px4_estimator_bias_subtracted"}
        if any(imu.get(key) != value for key, value in semantics.items()):
            errors.append("MAVLink HIGHRES_IMU sample endpoint/averaging/bias semantics must be recorded explicitly")
    if manifest.get("experimental") is not False:
        errors.append("Experimental or unspecified acquisition cannot be published as a formal calibration")
    if manifest.get("synthetic", False):
        errors.append("Synthetic data cannot be published as a real hardware calibration")
    if not camera_required:
        return errors
    for key in ("profile", "serial_no", "driver_version", "stream_mode", "exposure", "gain"):
        if key not in camera or camera[key] is None or camera[key] == "":
            errors.append(f"camera.{key} is missing")
    if camera.get("enable_auto_exposure") is not False:
        errors.append("camera.enable_auto_exposure must be false")
    if camera.get("stream_mode") not in ("raw", "rectified"):
        errors.append("camera.stream_mode must be raw or rectified")
    exposure, gain = camera.get("exposure"), camera.get("gain")
    if isinstance(exposure, bool) or not isinstance(exposure, (int, float)) or not np.isfinite(exposure) or exposure <= 0:
        errors.append("camera.exposure must be a measured positive value")
    if isinstance(gain, bool) or not isinstance(gain, (int, float)) or not np.isfinite(gain) or gain < 0:
        errors.append("camera.gain must be a measured nonnegative value")
    snapshot = camera.get("observed_snapshot", {})
    final_snapshot = camera.get("final_observed_snapshot", {})
    if snapshot.get("verified") is not True or final_snapshot.get("verified") is not True:
        errors.append("Camera startup/final driver readback snapshots are required")
    elif snapshot.get("parameters") != final_snapshot.get("parameters"):
        errors.append("Camera settings changed during acquisition")
    else:
        expected = {"serial_no": camera.get("serial_no"), "depth_module.infra_profile": camera.get("profile"),
                    "depth_module.exposure": exposure, "depth_module.gain": gain,
                    "depth_module.enable_auto_exposure": False, "depth_module.emitter_enabled": 0,
                    "enable_depth": False, "enable_color": False, "enable_gyro": False,
                    "enable_accel": False, "enable_infra1": True, "enable_infra2": True}
        if any(snapshot.get("parameters", {}).get(key) != value for key, value in expected.items()):
            errors.append("Observed camera parameters do not match the recorded rig")
        if snapshot.get("driver_version") != camera.get("driver_version") or final_snapshot.get("driver_version") != camera.get("driver_version"):
            errors.append("Observed camera driver version differs from the recorded rig")
    return errors


def session_provenance_errors(manifest, input_sha256, expected_mode, profile=DEFAULT_PROFILE):
    """Bind bringup's stopped bag file hashes to the actual offline input."""
    import hashlib
    errors = provenance_errors(manifest, camera_required=expected_mode == "joint", profile=profile)
    baseline = manifest.get("git", {})
    errors.extend(baseline_provenance_errors(baseline))
    if baseline.get("commit") != manifest.get("imu", {}).get("bridge_commit"):
        errors.append("Bridge revision differs from the acquisition Git baseline")
    if baseline.get("approved_patches", {}) != manifest.get("imu", {}).get("approved_source_patches", {}):
        errors.append("Approved patch provenance differs between bridge and acquisition baseline")
    if manifest.get("status") != "SUCCEEDED" or manifest.get("mode") != expected_mode:
        errors.append(f"Input requires a successful {expected_mode} acquisition manifest")
    files = manifest.get("evidence", {}).get("bag", {}).get("files_sha256")
    if not isinstance(files, dict) or not files:
        errors.append("Acquisition manifest lacks finalized bag file hashes")
    elif any(not isinstance(name, str) or not isinstance(digest, str) or len(digest) != 64 for name, digest in files.items()):
        errors.append("Acquisition bag file hash mapping is malformed")
    else:
        recorded_hash = hashlib.sha256(json.dumps(sorted(files.items()), separators=(",", ":")).encode()).hexdigest()
        if recorded_hash != input_sha256:
            errors.append("Input bag does not match the stopped acquisition manifest hashes")
    return errors


def baseline_provenance_errors(baseline):
    """Validate producer's recorded exception without treating arbitrary dirt as clean."""
    errors = []
    if baseline.get("effective_clean") is not True or baseline.get("provenance_errors"):
        errors.append("Implementation/acquisition Git baseline is not reproducibly clean")
    if not re.fullmatch(r"[0-9a-f]{40}", str(baseline.get("commit", ""))):
        errors.append("Implementation/acquisition Git commit is missing")
    if baseline.get("status"):
        try:
            from spr_diagnostics.run import (
                OPENVINS_APPROVED_PATCHES, OPENVINS_BASE,
                OPENVINS_COMBINED_DIFF_SHA256, OPENVINS_LEGACY_CXX20_PATCH,
                OPENVINS_PATCHED_FILES,
            )
        except ImportError as error:
            raise CalibrationError("Source diagnostics package is required to verify approved compatibility patches") from error
        patch = baseline.get("approved_patches", {}).get("open_vins", {})
        current_patch = {
            "base_commit": OPENVINS_BASE,
            "patches": [{"path": name, "sha256": digest}
                        for name, digest in OPENVINS_APPROVED_PATCHES.items()],
            "diff_sha256": OPENVINS_COMBINED_DIFF_SHA256,
            "patched_files_sha256": OPENVINS_PATCHED_FILES,
        }
        # Porcelain v1 deliberately reports submodule content changes as M;
        # unlike `git status --short`, it does not retain the lowercase m.
        # Historical recordings retain the exact reviewed C++20 provenance.
        # Accepting their metadata does not approve that patch for new captures.
        if (str(baseline["status"]).strip() != "M src/open_vins"
                or patch not in (current_patch, OPENVINS_LEGACY_CXX20_PATCH)):
            errors.append("Git dirt is not the exact reviewed OpenVINS compatibility exception")
    return errors


def hardware_equivalence_errors(training, heldout):
    errors = []
    if training.get("calibration_profile", DEFAULT_PROFILE) != heldout.get("calibration_profile", DEFAULT_PROFILE):
        errors.append("Held-out calibration_profile differs from the fitted rig")
    fields = {"camera": ("serial_no", "usb_serial_no", "profile", "stream_mode", "driver_version",
                         "exposure", "gain", "enable_auto_exposure", "left_image_topic", "right_image_topic"),
              "imu": ("source", "topic", "frame_id", "time_domain", "bridge_commit")}
    for group, keys in fields.items():
        for key in keys:
            if training.get(group, {}).get(key) != heldout.get(group, {}).get(key):
                errors.append(f"Held-out {group}.{key} differs from the fitted rig")
    if training.get("calibration_profile") == MAVLINK_PROFILE:
        for key in ("timestamp_source", "raw_imu", "integration_dt_available", "bias_policy"):
            if training.get("imu", {}).get(key) != heldout.get("imu", {}).get(key):
                errors.append(f"Held-out imu.{key} differs from the fitted MAVLink path")
    return errors


def covariance_report(path):
    covariance = np.load(path, allow_pickle=False)
    if covariance.ndim != 2 or covariance.shape[0] != covariance.shape[1] or covariance.shape[0] != 8:
        raise CalibrationError("Expected 8x8 joint covariance (6 extrinsic + 2 temporal parameters)")
    if not np.isfinite(covariance).all() or not np.allclose(covariance, covariance.T, atol=1e-7):
        raise CalibrationError("Joint covariance is not finite/symmetric")
    if np.min(np.linalg.eigvalsh(covariance)) < -1e-10 or np.any(np.diag(covariance) <= 0):
        raise CalibrationError("Joint covariance is not positive semidefinite with positive diagonal")
    std = np.sqrt(np.diag(covariance))
    correlation = covariance / np.outer(std, std)
    # The rotation parameterization is Kalibr's local tangent space.
    passed = np.max(std[:3]) <= np.deg2rad(2) and np.max(std[3:6]) <= 0.02 and np.max(std[6:]) <= 0.005
    return {"status": "PASSED" if passed else "FAILED",
            "ordering": ["rotation_x_rad", "rotation_y_rad", "rotation_z_rad",
                         "translation_x_m", "translation_y_m", "translation_z_m",
                         "cam0_timeshift_s", "cam1_timeshift_s"],
            "standard_deviation": std.tolist(), "covariance": covariance.tolist(),
            "correlation": correlation.tolist(),
            "limits": {"rotation_std_rad": float(np.deg2rad(2)), "translation_std_m": 0.02, "time_std_s": 0.005},
            "note": "Kalibr local linearized estimator covariance; systematic errors are not represented"}


def solve(bag, camchain, imu, output, image=DEFAULT_KALIBR_IMAGE, session_manifest=None, max_iter=30,
          profile=DEFAULT_PROFILE, *, noise_prior=None):
    noise = imu_config(read_yaml(imu), profile)
    noise_annotations = {"noise_basis": "provided_unverified", "noise_model_measured": False,
                         "original_allan_requirement_satisfied": False}
    if noise_prior is not None:
        noise_annotations = validate_noise_prior(read_yaml(noise_prior), noise, profile)
        noise_prior_sha256 = sha256_file(noise_prior)
    try:
        from spr_diagnostics.run import git_baseline
    except ImportError as error:
        raise CalibrationError("Source the workspace diagnostics package before running the solver") from error
    implementation = git_baseline(Path.cwd())
    chain = camera_chain(read_yaml(camchain))
    contract = profile_config(profile)
    source_manifest = read_yaml(session_manifest) if session_manifest else {}
    input_sha256 = hash_input(bag)
    errors = session_provenance_errors(source_manifest, input_sha256, "joint", profile)
    errors.extend(baseline_provenance_errors(implementation))
    if Path(bag).is_dir() and Path(output).resolve().is_relative_to(Path(bag).resolve()):
        raise CalibrationError("Solver output cannot be inside the source bag")
    image_info = subprocess.run(["docker", "image", "inspect", image], text=True, capture_output=True, check=True)
    image_id = json.loads(image_info.stdout)[0]["Id"]
    if not image_id.startswith("sha256:"):
        raise CalibrationError("Docker did not resolve Kalibr to an immutable image ID")
    version = subprocess.run(["docker", "run", "--rm", "--network", "none", "--entrypoint", "git",
                              image_id, "-C", "/catkin_ws/src/kalibr", "rev-parse", "HEAD"],
                             text=True, capture_output=True, check=True).stdout.strip()
    if len(version) != 40 or any(char not in "0123456789abcdef" for char in version):
        raise CalibrationError("Kalibr source commit cannot be verified in the selected image")
    output = new_output(output)
    manifest = {"schema_version": 1, "status": "RUNNING", "created_utc": utc_now(),
                "calibration_profile": profile,
                "deployment_scope": contract.get("deployment_scope", "dds_runtime"),
                "git_commit": implementation["commit"] or "UNCOMMITTED", "git": implementation,
                "input_bag": str(Path(bag).resolve()),
                "input_bag_sha256": input_sha256, "image_id": image_id,
                "kalibr_commit": version,
                "tool_versions": tool_versions(),
                "camera": source_manifest.get("camera", {}), "imu": source_manifest.get("imu", {}),
                "formal_provenance_errors": errors, "synthetic": source_manifest.get("synthetic", False),
                "experimental": source_manifest.get("experimental", True),
                "source_run_id": source_manifest.get("run_id"),
                "source_manifest_sha256": sha256_file(session_manifest) if session_manifest else None,
                "camchain_input_sha256": sha256_file(camchain), "imu_input_sha256": sha256_file(imu),
                "time_convention": "t_imu = t_cam + timeshift_cam_imu"}
    manifest.update(profile_annotations(profile))
    manifest.update(noise_annotations)
    if noise_prior is not None:
        manifest["noise_prior_input_sha256"] = noise_prior_sha256
    manifest["header_time_policy"] = (f"host_ros already produced by {contract['source']}; "
                                      "conversion preserves header timestamps without applying T_host_px4 again")
    if profile == MAVLINK_PROFILE:
        manifest["timestamp_semantics"] = {
            "mapping": "host_ns = reference_host_ns + clock_scale * (px4_ns - reference_px4_ns)",
            "offset_scope": "T_host_px4_offset_s is local at the recorded clock reference, not a constant over the recording",
            "physical_sample_time": "PX4 gyro sample endpoint; integration midpoint/accel sample time unavailable in HIGHRES_IMU",
            "uncertainty_scope": "TIMESYNC fit/RTT bounds describe clock mapping, not physical IMU/camera synchronization"}
    write_yaml(output / "manifest.yaml", manifest)
    try:
        write_yaml(output / "camchain.yaml", chain)
        write_yaml(output / "imu.yaml", noise)
        write_yaml(output / "target.yaml", target_config())
        if noise_prior is not None:
            shutil.copyfile(noise_prior, output / "noise_prior.yaml")
            if sha256_file(output / "noise_prior.yaml") != noise_prior_sha256:
                raise CalibrationError("Noise prior changed after input validation")
            validate_noise_prior(read_yaml(output / "noise_prior.yaml"), noise, profile)
        if session_manifest:
            shutil.copyfile(session_manifest, output / "session_manifest.yaml")
        topics = {chain[name]["rostopic"]: "sensor_msgs/msg/Image" for name in ("cam0", "cam1")}
        topics[noise["rostopic"]] = "sensor_msgs/msg/Imu"
        conversion = convert_bag(bag, output / "input.bag", topics)
        conversion.update(profile_annotations(profile))
        write_yaml(output / "conversion.yaml", conversion)
        observed_imu = conversion["source"]["topics"][noise["rostopic"]]
        if observed_imu["frame_id"] != "imu_link":
            raise CalibrationError(f"{noise['rostopic']} header.frame_id must be imu_link")
        actual_imu_rate = observed_imu["rate_hz"]
        if profile == MAVLINK_PROFILE and actual_imu_rate < contract["min_rate_hz"]:
            raise CalibrationError(f"MAVLink motion bag must actually contain >={contract['min_rate_hz']:g} Hz HIGHRES_IMU")
        if abs(actual_imu_rate - noise["update_rate"]) > noise["update_rate"] * 0.1:
            raise CalibrationError("Motion bag IMU rate differs by >10% from declared IMU model update_rate")
        for name in ("cam0", "cam1"):
            actual = conversion["source"]["topics"][chain[name]["rostopic"]]["profile"][:2]
            if actual != chain[name]["resolution"]:
                raise CalibrationError(f"{name}: bag resolution differs from camera calibration")
        shutil.copyfile(Path(__file__).with_name("kalibr_covariance.py"), output / "kalibr_covariance_entry.py")
        command = ["docker", "run", "--rm", "--network", "none", "--user", f"{os.getuid()}:{os.getgid()}",
                   "--entrypoint", "/bin/bash", "-e", "MPLBACKEND=Agg", "-e", "HOME=/tmp",
                   "-v", f"{output}:/data", image_id, "-lc",
                   'source /catkin_ws/devel/setup.bash && exec "$@"', "bash",
                   "python3", "/data/kalibr_covariance_entry.py", "--bag", "/data/input.bag",
                   "--cams", "/data/camchain.yaml", "--imu", "/data/imu.yaml", "--target", "/data/target.yaml",
                   "--imu-models", "calibrated", "--recover-covariance", "--dont-show-report",
                   "--max-iter", str(max_iter)]
        write_yaml(output / "command.yaml", {"argv": command, "image_id": image_id})
        with (output / "kalibr.log").open("w", encoding="utf-8") as log:
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
            try:
                stream_kalibr_output(process.stdout, log)
                returncode = process.wait()
            except BaseException:
                process.terminate()
                process.wait()
                raise
        if returncode:
            raise CalibrationError(f"Kalibr exited {returncode}; see {output / 'kalibr.log'}")
        # Names verified against the pinned Kalibr source, which prefixes each
        # artifact with the input bag stem rather than the wiki's old naming.
        result = output / "input-camchain-imucam.yaml"
        pdf = output / "input-report-imucam.pdf"
        if not result.is_file() or not pdf.is_file() or pdf.stat().st_size < 1000:
            raise CalibrationError("Kalibr did not produce joint YAML and a PDF report")
        final_chain = camera_chain(read_yaml(result), joint=True)
        write_yaml(output / "kalibr_imucam_chain.yaml", final_chain)
        shutil.copyfile(pdf, output / "report.pdf")
        covariance = covariance_report(output / "joint_covariance.npy")
        write_yaml(output / "uncertainty.yaml", covariance)
        manifest.update(status="CANDIDATE", uncertainty_status=covariance["status"],
                        result_sha256=sha256_file(output / "kalibr_imucam_chain.yaml"),
                        report_sha256=sha256_file(output / "report.pdf"),
                        completed_utc=utc_now())
        manifest["artifact_sha256"] = {name: sha256_file(output / name) for name in (
            "camchain.yaml", "imu.yaml", "target.yaml", "kalibr_imucam_chain.yaml",
            "joint_covariance.npy", "covariance_method.yaml", "covariance_structure.yaml", "uncertainty.yaml", "report.pdf", "conversion.yaml",
            "command.yaml", "kalibr_covariance_entry.py")}
        if session_manifest:
            manifest["artifact_sha256"]["session_manifest.yaml"] = sha256_file(output / "session_manifest.yaml")
        if noise_prior is not None:
            manifest["artifact_sha256"]["noise_prior.yaml"] = sha256_file(output / "noise_prior.yaml")
    except BaseException as error:
        manifest.update(status="FAILED", error=str(error), completed_utc=utc_now())
        raise
    finally:
        write_yaml(output / "manifest.yaml", manifest)
        report_html(output / "report.html", "Kalibr candidate calibration", manifest)
    return manifest
