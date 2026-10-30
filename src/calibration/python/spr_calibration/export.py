"""OpenVINS final dialect export and immutable, evidence-gated publication."""

from datetime import datetime, timezone
from pathlib import Path
import shutil

import cv2
import numpy as np
import yaml

from .common import (CalibrationError, NOISE_KEYS, camera_chain, hash_input,
                     imu_config, read_yaml, report_html, sha256_file, transform,
                     utc_now, write_yaml)
from .solver import baseline_provenance_errors, covariance_report, hardware_equivalence_errors, provenance_errors, session_provenance_errors
from .profiles import DEFAULT_PROFILE, MAVLINK_PROFILE, profile_annotations, profile_config, require_candidate_profile
from .bags import inspect_bag
from .validation import validate_stream_independence
from .noise import validate_noise_prior


def _opencv_yaml(path, data):
    # OpenCV FileStorage does not support PyYAML's indentless sequences.
    class IndentedDumper(yaml.SafeDumper):
        def increase_indent(self, flow=False, indentless=False):
            return super().increase_indent(flow, False)

        def ignore_aliases(self, data):
            # OpenCV FileStorage cannot resolve YAML anchors/aliases.
            return True
    def sequence(dumper, value):
        return dumper.represent_sequence("tag:yaml.org,2002:seq", value,
                                         flow_style=all(not isinstance(item, (list, dict)) for item in value))
    IndentedDumper.add_representer(list, sequence)
    Path(path).write_text("%YAML:1.0\n---\n" + yaml.dump(data, Dumper=IndentedDumper,
                         sort_keys=False, default_flow_style=False), encoding="utf-8")


def prepare_openvins(candidate, estimator_template, profile=DEFAULT_PROFILE):
    """Create candidate-local replay inputs. This does not publish a calibration."""
    candidate = Path(candidate).resolve()
    manifest = read_yaml(candidate / "manifest.yaml")
    require_candidate_profile(manifest, profile)
    if manifest.get("status") != "CANDIDATE":
        raise CalibrationError("Only a completed Kalibr CANDIDATE can be exported for replay")
    chain_path = candidate / "kalibr_imucam_chain.yaml"
    if sha256_file(chain_path) != manifest.get("result_sha256"):
        raise CalibrationError("Candidate calibration changed after Kalibr completed")
    chain = camera_chain(read_yaml(chain_path), joint=True)
    if abs(chain["cam0"]["timeshift_cam_imu"] - chain["cam1"]["timeshift_cam_imu"]) > 0.001:
        raise CalibrationError("OpenVINS has one stereo time offset; cam0/cam1 differ by >1 ms")
    noise = imu_config(read_yaml(candidate / "imu.yaml"), profile)
    noise_annotations = {}
    if manifest.get("noise_basis") == "engineering_prior":
        _, noise_annotations = validate_candidate_noise(candidate, manifest, profile)
    template_text = Path(estimator_template).read_text(encoding="utf-8")
    # This is the intentional final OpenCV dialect boundary.
    template = yaml.safe_load("\n".join(line for line in template_text.splitlines() if not line.startswith("%YAML:")))
    if not isinstance(template, dict):
        raise CalibrationError("Estimator template must contain a YAML mapping")
    template.update(max_cameras=2, use_stereo=True, calib_cam_timeoffset=True,
                    calib_imu_intrinsics=False, calib_imu_g_sensitivity=False,
                    relative_config_imu="kalibr_imu_chain.yaml",
                    relative_config_imucam="kalibr_imucam_chain.yaml")
    output = candidate / "openvins"
    output.mkdir(exist_ok=False)
    try:
        _opencv_yaml(output / "estimator_config.yaml", template)
        # Retain Kalibr's T_cam_imu and sign; OpenVINS inverts this as required.
        _opencv_yaml(output / "kalibr_imucam_chain.yaml", chain)
        identity = np.eye(3).tolist()
        inertial = dict(noise, T_i_b=np.eye(4).tolist(), time_offset=0.0, model="kalibr",
                        Tw=identity, Ta=identity, R_IMUtoGYRO=identity, R_IMUtoACC=identity,
                        Tg=np.zeros((3, 3)).tolist())
        _opencv_yaml(output / "kalibr_imu_chain.yaml", {"imu0": inertial})
        for path in output.glob("*.yaml"):
            storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
            if not storage.isOpened():
                raise CalibrationError(f"OpenCV cannot parse final export {path.name}")
            storage.release()
        files = {path.name: sha256_file(path) for path in sorted(output.glob("*.yaml"))}
        write_yaml(candidate / "openvins_export.yaml", {
            "status": "CANDIDATE_REPLAY_INPUT", "result_sha256": sha256_file(chain_path),
            **profile_annotations(profile),
            **noise_annotations,
            "deployment_scope": manifest.get("deployment_scope", "dds_runtime"),
            "template_sha256": sha256_file(estimator_template), "files": files,
            "config_sha256": hash_input(output), "created_utc": utc_now(),
            "convention": "T_cam_imu maps IMU to camera; t_imu=t_cam+timeshift_cam_imu"})
    except Exception:
        shutil.rmtree(output)
        raise
    return output


def _evidence_file(item, base_directory, label):
    if not isinstance(item, dict) or not isinstance(item.get("path"), str) or not item.get("sha256"):
        raise CalibrationError(f"Replay {label} requires path and sha256")
    path = Path(item["path"])
    if not path.is_absolute():
        path = Path(base_directory) / path
    path = path.resolve()
    if not path.is_file() or path.stat().st_size == 0 or sha256_file(path) != item["sha256"]:
        raise CalibrationError(f"Replay {label} artifact is absent, empty or has changed")
    return path


def _inspect_replay_streams(bag, training_topics, profile):
    """Verify stream equivalence/independence without claiming hardware provenance."""
    contract = profile_config(profile)
    imu_topic = contract["imu_topic"]
    if not isinstance(training_topics, dict) or not all(isinstance(item, dict) for item in training_topics.values()):
        raise CalibrationError("Replay requires the training bag topic inspection")
    topics = {topic: item.get("type") for topic, item in training_topics.items()}
    if (topics.get(imu_topic) != "sensor_msgs/msg/Imu" or len(topics) != 3
            or sum(kind == "sensor_msgs/msg/Image" for kind in topics.values()) != 2):
        raise CalibrationError(f"Replay training inspection must contain two images and {imu_topic}")
    inspection = inspect_bag(bag, topics)
    validate_stream_independence(inspection["topics"], training_topics, imu_topic, label="Replay")
    for topic, observed in inspection["topics"].items():
        rates = [item.get("rate_hz") for item in (training_topics[topic], observed)]
        if any(isinstance(rate, bool) or not isinstance(rate, (int, float))
               or not np.isfinite(rate) or rate <= 0 for rate in rates):
            raise CalibrationError(f"{topic}: measured training/replay rate is missing or invalid")
        minimum_rate = contract.get("min_rate_hz", 200.0)
        if topic == imu_topic and min(rates) < minimum_rate:
            raise CalibrationError(f"{profile} training/replay IMU streams must actually contain >={minimum_rate:g} Hz")
    return inspection


def validate_replay_evidence(evidence, expected_config_hash, training_hash, base_directory=None,
                             *, training_topics=None, profile=None):
    if evidence.get("status") != "PASSED" or evidence.get("config_sha256") != expected_config_hash:
        raise CalibrationError("Replay evidence must pass and bind the exact OpenVINS config hash")
    if not evidence.get("bag_sha256") or evidence.get("bag_sha256") == training_hash:
        raise CalibrationError("Replay must use an independently recorded held-out bag")
    if evidence.get("diverged") is not False or not evidence.get("run_id"):
        raise CalibrationError("Replay evidence requires run_id and diverged:false")
    reference = evidence.get("reference", {})
    metrics = evidence.get("metrics", {})
    if reference.get("source") == "none":
        if any("ate" in key.lower() or "rpe" in key.lower() for key in metrics):
            raise CalibrationError("ATE/RPE without a truth reference are forbidden")
        for key in ("closed_loop_drift_m", "repeatability_m"):
            value = metrics.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value < 0:
                raise CalibrationError(f"No-truth replay requires measured {key}")
        if evidence.get("proxy_acceptance") is not True:
            raise CalibrationError("No-truth proxy acceptance must be explicitly recorded by the reviewer")
        acceptance = evidence.get("proxy_limits", {})
        for metric in ("closed_loop_drift_m", "repeatability_m"):
            limit = acceptance.get(metric)
            if isinstance(limit, bool) or not isinstance(limit, (int, float)) or not np.isfinite(limit) or limit <= 0:
                raise CalibrationError(f"A reviewed positive proxy_limits.{metric} is required")
            if metrics[metric] > limit:
                raise CalibrationError(f"Replay {metric} exceeds its reviewed proxy limit")
    else:
        if reference.get("source") not in ("sitl_gt", "mocap", "apriltag_field", "total_station") or not reference.get("accuracy"):
            raise CalibrationError("Replay reference source/accuracy is missing or unsupported")
        ate = metrics.get("ate_rmse_m")
        if isinstance(ate, bool) or not isinstance(ate, (float, int)) or not np.isfinite(ate) or not 0 <= ate <= 0.3:
            raise CalibrationError("Ground-truth replay requires ATE RMSE <=0.3 m")
    if base_directory is None:
        raise CalibrationError("Replay evidence must resolve its actual recorded artifacts")
    if training_topics is None or profile is None:
        raise CalibrationError("Replay requires training topic inspection and an explicitly selected calibration profile")
    artifacts = evidence.get("artifacts", {})
    required = ["estimated_trajectory", "replay_log", "replay_command", "evaluation_report"]
    if reference.get("source") != "none":
        required.append("reference_trajectory")
    else:
        required.append("acceptance_adr")
    resolved = {name: _evidence_file(artifacts.get(name), base_directory, name) for name in required}
    command = read_yaml(resolved["replay_command"])
    if not isinstance(command.get("argv"), list) or not command["argv"] or any(not isinstance(arg, str) for arg in command["argv"]):
        raise CalibrationError("Replay command artifact must contain the actual nonempty argv list")
    bag_path = evidence.get("bag_path")
    if not isinstance(bag_path, str):
        raise CalibrationError("Replay evidence must point to the stopped bag that was actually evaluated")
    replay_bag = Path(bag_path)
    if not replay_bag.is_absolute():
        replay_bag = Path(base_directory) / replay_bag
    inspection = _inspect_replay_streams(replay_bag, training_topics, profile)
    if inspection["input_sha256"] != evidence["bag_sha256"]:
        raise CalibrationError("Replay bag does not match the evaluated input hash")
    evidence["selected_topic_inspection"] = inspection["topics"]
    return resolved


def validate_candidate_noise(candidate, manifest, profile=DEFAULT_PROFILE):
    """Verify measured Allan evidence or an explicit MAVLink-only model prior.

    The latter validates declaration and integrity, never a noise measurement.
    Its flags remain visible in replay inputs and published test artifacts.
    """
    candidate = Path(candidate)
    prior = manifest.get("noise_basis") == "engineering_prior"
    filename = "noise_prior.yaml" if prior else "allan_report.yaml"
    expected = manifest.get("artifact_sha256", {}).get(filename)
    path = candidate / filename
    if not expected or not path.is_file() or sha256_file(path) != expected:
        raise CalibrationError(f"Candidate noise evidence is missing or changed: {filename}")
    noise = imu_config(read_yaml(candidate / "imu.yaml"), profile)
    document = read_yaml(path)
    if prior:
        annotations = validate_noise_prior(document, noise, profile)
        if any(type(manifest.get(key)) is not type(value) or manifest[key] != value
               for key, value in annotations.items()):
            raise CalibrationError("Candidate must preserve the unmeasured engineering noise prior flags")
        return filename, annotations
    if (document.get("status") != "PASSED" or document.get("formal_duration") is not True
            or document.get("formal_eligible") is not True):
        raise CalibrationError("Measured stationary >=1 hour Allan analysis is required")
    require_candidate_profile(document, profile)
    source = document.get("acquisition_manifest", {})
    errors = session_provenance_errors(source, document.get("input_bag_sha256"), "allan", profile)
    if errors or source.get("imu", {}).get("bridge_commit") != manifest["imu"]["bridge_commit"]:
        raise CalibrationError("Allan acquisition must use the same clean committed bridge version")
    if any(not np.isclose(noise[key], document.get("imu", {}).get(key, float("nan")), rtol=1e-8)
           for key in NOISE_KEYS):
        raise CalibrationError("IMU noise differs from measured Allan result")
    return filename, {}


def publish(candidate, replay_evidence, destination_root, profile=DEFAULT_PROFILE):
    """Publish only validated real data; candidate paths are never runtime defaults."""
    candidate = Path(candidate).resolve()
    manifest = read_yaml(candidate / "manifest.yaml")
    require_candidate_profile(manifest, profile)
    if manifest.get("status") != "CANDIDATE":
        raise CalibrationError("Kalibr candidate is incomplete or failed")
    errors = provenance_errors(manifest, profile=profile)
    errors.extend(baseline_provenance_errors(manifest.get("git", {})))
    if manifest.get("git", {}).get("commit") != manifest.get("git_commit"):
        errors.append("Candidate solver commit differs from its implementation baseline")
    errors += manifest.get("formal_provenance_errors", [])
    if errors:
        raise CalibrationError("Formal provenance failed: " + "; ".join(sorted(set(errors))))
    hashes = manifest.get("artifact_sha256", {})
    noise_filename = "noise_prior.yaml" if manifest.get("noise_basis") == "engineering_prior" else "allan_report.yaml"
    if not {"camchain.yaml", "imu.yaml", "kalibr_imucam_chain.yaml", "joint_covariance.npy",
            "uncertainty.yaml", "report.pdf", "conversion.yaml", noise_filename}.issubset(hashes):
        raise CalibrationError("Candidate lacks hashes for required calibration artifacts")
    for name, expected_hash in hashes.items():
        if sha256_file(candidate / name) != expected_hash:
            raise CalibrationError(f"Candidate artifact changed: {name}")
    acquisition = read_yaml(candidate / "session_manifest.yaml")
    if sha256_file(candidate / "session_manifest.yaml") != manifest.get("source_manifest_sha256"):
        raise CalibrationError("Source acquisition manifest changed after fitting")
    acquisition_errors = session_provenance_errors(acquisition, manifest["input_bag_sha256"], "joint", profile)
    if acquisition_errors:
        raise CalibrationError("Source acquisition provenance failed: " + "; ".join(acquisition_errors))
    chain_path = candidate / "kalibr_imucam_chain.yaml"
    camera_chain(read_yaml(chain_path), joint=True)
    if sha256_file(chain_path) != manifest.get("result_sha256"):
        raise CalibrationError("Joint camera calibration changed since solve")
    uncertainty = read_yaml(candidate / "uncertainty.yaml")
    if uncertainty.get("status") != "PASSED" or covariance_report(candidate / "joint_covariance.npy")["status"] != "PASSED":
        raise CalibrationError("Kalibr covariance uncertainty gates failed")
    selected = read_yaml(candidate / "validation" / "selected.yaml")
    attempt = selected.get("attempt")
    if not isinstance(attempt, str) or Path(attempt).name != attempt or not attempt.startswith("attempt_"):
        raise CalibrationError("Selected held-out attempt is invalid")
    heldout_path = candidate / "validation" / attempt / "heldout.yaml"
    if sha256_file(heldout_path) != selected.get("heldout_sha256"):
        raise CalibrationError("Selected held-out result changed")
    heldout = read_yaml(heldout_path)
    if heldout.get("status") != "PASSED" or heldout.get("result_sha256") != manifest["result_sha256"]:
        raise CalibrationError("Independent held-out stereo/IMU validation is missing or failed")
    heldout_source = heldout.get("acquisition_manifest", {})
    heldout_errors = session_provenance_errors(heldout_source, heldout.get("heldout_bag_sha256"), "joint", profile)
    heldout_errors.extend(hardware_equivalence_errors(manifest, heldout_source))
    if heldout.get("formal_eligible") is not True or heldout_errors:
        raise CalibrationError("Held-out acquisition provenance or rig equivalence is unverified")
    if (heldout.get("time_sign", {}).get("status") != "PASSED"
            or heldout.get("accelerometer", {}).get("status") != "PASSED"
            or heldout.get("detected_pairs", 0) < 30
            or heldout.get("stereo_pair_ratio", 0) < 0.95
            or heldout.get("reprojection_rms_px", float("inf")) > 0.5
            or heldout.get("max_view_rms_px", float("inf")) > 1.0):
        raise CalibrationError("Held-out stereo, time sign, or inertial quality gates failed")
    if heldout.get("heldout_bag_sha256") == manifest["input_bag_sha256"]:
        raise CalibrationError("Held-out calibration data must differ from fitting data")
    noise_filename, noise_annotations = validate_candidate_noise(candidate, manifest, profile)
    export = read_yaml(candidate / "openvins_export.yaml")
    require_candidate_profile(export, profile)
    if any(type(export.get(key)) is not type(value) or export[key] != value
           for key, value in noise_annotations.items()):
        raise CalibrationError("OpenVINS replay input lost the engineering noise prior flags")
    if hash_input(candidate / "openvins") != export.get("config_sha256"):
        raise CalibrationError("OpenVINS files changed after replay preparation")
    evidence = read_yaml(replay_evidence)
    if profile == MAVLINK_PROFILE:
        require_candidate_profile(evidence, profile)
    training_topics = read_yaml(candidate / "conversion.yaml")["source"]["topics"]
    replay_artifacts = validate_replay_evidence(
        evidence, export["config_sha256"], manifest["input_bag_sha256"],
        Path(replay_evidence).resolve().parent, training_topics=training_topics, profile=profile)
    original_bag_path = Path(evidence["bag_path"])
    if not original_bag_path.is_absolute():
        original_bag_path = Path(replay_evidence).resolve().parent / original_bag_path
    evidence["bag_path"] = str(original_bag_path.resolve())
    root = Path(destination_root).resolve()
    root.mkdir(parents=True, exist_ok=True)
    date = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    commit = str(manifest["git_commit"])[:8]
    if commit == "UNCOMMIT":
        raise CalibrationError("Commit the tested implementation before publishing a formal calibration")
    calibration_id = f"{date}_{commit}_{manifest['input_bag_sha256'][:8]}"
    if profile == MAVLINK_PROFILE:
        calibration_id = profile + "_" + calibration_id
    output = root / calibration_id
    output.mkdir(exist_ok=False)
    try:
        for name in ("camchain.yaml", "imu.yaml", "kalibr_imucam_chain.yaml", "report.pdf",
                     "uncertainty.yaml", "joint_covariance.npy", noise_filename,
                     "conversion.yaml", "target.yaml", "command.yaml", "kalibr.log", "kalibr_covariance_entry.py",
                     "session_manifest.yaml"):
            shutil.copyfile(candidate / name, output / name)
        for diagnostic in ("covariance_method.yaml", "covariance_structure.yaml"):
            if diagnostic in hashes:
                shutil.copyfile(candidate / diagnostic, output / diagnostic)
        shutil.copytree(candidate / "openvins", output / "openvins")
        shutil.copytree(candidate / "validation", output / "validation")
        artifact_root = output / "validation" / "replay_artifacts"
        artifact_root.mkdir()
        for name, original in replay_artifacts.items():
            destination = artifact_root / (name + original.suffix)
            shutil.copyfile(original, destination)
            evidence["artifacts"][name]["path"] = "replay_artifacts/" + destination.name
        write_yaml(output / "validation" / "replay_metrics.yaml", evidence)
        final = dict(manifest, status="VALIDATED", calibration_id=calibration_id,
                     published_utc=utc_now(), openvins_config_sha256=export["config_sha256"])
        final["files"] = {str(path.relative_to(output)): sha256_file(path)
                          for path in sorted(output.rglob("*")) if path.is_file()}
        write_yaml(output / "manifest.yaml", final)
        report_html(output / "validation" / "report.html", f"Calibration {calibration_id}",
                    {"status": "PASSED", "manifest": final, "heldout": heldout, "replay": evidence})
    except Exception:
        shutil.rmtree(output)
        raise
    return output
