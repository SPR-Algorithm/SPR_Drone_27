"""Command-line entry for the offline FC IMU + D435i calibration workflow."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import sys

from .allan import analyze_allan
from .bags import convert_bag, inspect_bag, load_imu
from .common import (CalibrationError, camera_chain, hash_input, new_output,
                     read_yaml, report_html, sha256_file, tool_versions, write_yaml)
from .export import prepare_openvins, publish
from .noise import validate_noise_prior
from .solver import DEFAULT_KALIBR_IMAGE, hardware_equivalence_errors, session_provenance_errors, solve
from .profiles import (DEFAULT_PROFILE, MAVLINK_PROFILE, PROFILES, profile_annotations,
                       profile_config, require_candidate_profile)
from .validation import validate_heldout


def topics_for(args):
    imu_topic = profile_config(args.profile)["imu_topic"]
    if args.left_topic == args.right_topic or imu_topic in (args.left_topic, args.right_topic):
        raise CalibrationError(f"Two distinct image topics, separate from {imu_topic}, are required")
    return {args.left_topic: "sensor_msgs/msg/Image", args.right_topic: "sensor_msgs/msg/Image",
            imu_topic: "sensor_msgs/msg/Imu"}


def parser():
    cli = argparse.ArgumentParser(description="Offline D435i stereo + FC IMU calibration (8x11 inner corners, 25 mm)")
    sub = cli.add_subparsers(dest="command", required=True)
    for name in ("inspect", "convert"):
        command = sub.add_parser(name, help="Check all messages and timestamps" if name == "inspect" else "ROS2/ROS1 input -> verified Kalibr ROS1 bag")
        command.add_argument("--bag", required=True, type=Path)
        command.add_argument("--left-topic", default="/camera/infra1/image_rect_raw")
        command.add_argument("--right-topic", default="/camera/infra2/image_rect_raw")
        if name == "convert":
            command.add_argument("--output", required=True, type=Path, help="New .bag file (will not overwrite)")
    command = sub.add_parser("allan", help="Measured stationary Allan analysis; >=1h required for formal use")
    command.add_argument("--bag", type=Path, required=True)
    command.add_argument("--output", type=Path, required=True, help="New run directory")
    command.add_argument("--session-manifest", type=Path, help="Bringup's finalized static acquisition manifest")
    command.add_argument("--experimental-min-duration", type=float, default=3600,
                         help="Diagnostics only; <3600s cannot produce a formal calibration")
    command = sub.add_parser("solve", help="Run real Kalibr with temporal calibration and covariance recovery")
    for option in ("bag", "camchain", "imu", "output"):
        command.add_argument("--" + option, type=Path, required=True)
    command.add_argument("--session-manifest", type=Path)
    noise_evidence = command.add_mutually_exclusive_group()
    noise_evidence.add_argument("--allan-report", type=Path, help="Measured Allan report from this tool")
    noise_evidence.add_argument("--noise-prior", type=Path,
                                help="Source-attributed engineering prior; MAVLink test phase only, unmeasured")
    command.add_argument("--image", default=DEFAULT_KALIBR_IMAGE, help="Local image; resolved to sha256 before execution")
    command.add_argument("--max-iter", type=int, default=30)
    command = sub.add_parser("validate", help="Independent fixed-target held-out stereo + gyro/accel/time-sign checks")
    command.add_argument("--candidate", type=Path, required=True)
    command.add_argument("--bag", type=Path, required=True)
    command.add_argument("--session-manifest", type=Path, help="Independent held-out acquisition manifest; required for formal acceptance")
    command = sub.add_parser("prepare-openvins", help="Write candidate-local OpenVINS inputs for replay; does not publish")
    command.add_argument("--candidate", type=Path, required=True)
    command.add_argument("--estimator-template", type=Path, required=True)
    command = sub.add_parser("export", help="Publish validated real calibration after held-out and VIO replay gates")
    command.add_argument("--candidate", type=Path, required=True)
    command.add_argument("--replay-evidence", type=Path, required=True)
    command.add_argument("--destination", type=Path, default=Path("calibrations"))
    for command in sub.choices.values():
        command.add_argument("--profile", choices=tuple(PROFILES), default=DEFAULT_PROFILE,
                             help="Explicit acquisition contract; MAVLink results cannot be deployed as DDS calibration")
    return cli


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        contract = profile_config(args.profile)
        imu_topic = contract["imu_topic"]
        if args.command == "inspect":
            result = inspect_bag(args.bag, topics_for(args))
            result.update(profile_annotations(args.profile))
        elif args.command == "convert":
            result = convert_bag(args.bag, args.output, topics_for(args))
            result.update(profile_annotations(args.profile))
        elif args.command == "allan":
            output = new_output(args.output)
            try:
                inspected = inspect_bag(args.bag, {imu_topic: "sensor_msgs/msg/Imu"})
                observed_imu = inspected["topics"][imu_topic]
                if observed_imu["frame_id"] != "imu_link":
                    raise CalibrationError(f"Allan recording {imu_topic} must have frame_id imu_link")
                if args.profile == MAVLINK_PROFILE and observed_imu["rate_hz"] < contract["min_rate_hz"]:
                    raise CalibrationError(f"MAVLink Allan bag must actually contain >={contract['min_rate_hz']:g} Hz HIGHRES_IMU")
                ns, values = load_imu(args.bag, imu_topic)
                result = analyze_allan(ns, values, args.experimental_min_duration, imu_topic)
                result.update(input_bag_sha256=inspected["input_sha256"], input_bag=str(args.bag.resolve()),
                              topic_inspection=inspected["topics"], tool_versions=tool_versions(),
                              **profile_annotations(args.profile))
                source_manifest = read_yaml(args.session_manifest) if args.session_manifest else {}
                errors = session_provenance_errors(source_manifest, inspected["input_sha256"], "allan", args.profile)
                result.update(acquisition_manifest=source_manifest, formal_provenance_errors=errors,
                              formal_eligible=result.get("status") == "PASSED" and result["formal_duration"] and not errors)
                if result.get("status") == "PASSED":
                    result.update(noise_basis="measured_allan", noise_model_measured=True,
                                  original_allan_requirement_satisfied=result["formal_duration"])
            except CalibrationError as error:
                result = {"status": "FAILED", "error": str(error)}
            write_yaml(output / "allan_report.yaml", result)
            report_html(output / "report.html", "Stationary IMU Allan analysis", result)
            if result.get("status") == "PASSED":
                write_yaml(output / "imu.yaml", result["imu"])
            else:
                print(json.dumps(result, ensure_ascii=False))
                return 1
        elif args.command == "solve":
            if args.max_iter < 1:
                raise CalibrationError("--max-iter must be positive")
            if args.noise_prior:
                validate_noise_prior(read_yaml(args.noise_prior), read_yaml(args.imu), args.profile)
            if args.allan_report:
                measured = read_yaml(args.allan_report)
                require_candidate_profile(measured, args.profile)
                if measured.get("status") != "PASSED" or measured.get("formal_duration") is not True:
                    raise CalibrationError("Supplied Allan report is not a passed >=1h measurement")
                if measured.get("imu") != read_yaml(args.imu):
                    raise CalibrationError("--imu does not match --allan-report measured noise")
            result = solve(args.bag, args.camchain, args.imu, args.output, args.image,
                           args.session_manifest, args.max_iter, args.profile, noise_prior=args.noise_prior)
            if args.allan_report:
                shutil.copyfile(args.allan_report, args.output / "allan_report.yaml")
                result["artifact_sha256"]["allan_report.yaml"] = sha256_file(args.output / "allan_report.yaml")
                result["formal_provenance_errors"].extend(measured.get("formal_provenance_errors", ["Allan provenance was not recorded"]))
                result.update(noise_basis="measured_allan", noise_model_measured=True,
                              original_allan_requirement_satisfied=True)
                write_yaml(args.output / "manifest.yaml", result)
                report_html(args.output / "report.html", "Kalibr candidate calibration", result)
        elif args.command == "validate":
            manifest = read_yaml(args.candidate / "manifest.yaml")
            require_candidate_profile(manifest, args.profile)
            chain_path = args.candidate / "kalibr_imucam_chain.yaml"
            if manifest.get("status") != "CANDIDATE" or sha256_file(chain_path) != manifest.get("result_sha256"):
                raise CalibrationError("Candidate is incomplete or its result changed")
            validation_root = args.candidate / "validation"
            validation_root.mkdir(exist_ok=True)
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
            validation = validation_root / ("attempt_" + stamp)
            validation.mkdir(exist_ok=False)
            try:
                training = read_yaml(args.candidate / "conversion.yaml")["source"]["topics"]
                tolerance = manifest.get("camera", {}).get("pair_tolerance_s", 0.001)
                result = validate_heldout(args.bag, camera_chain(read_yaml(chain_path), True), manifest["input_bag_sha256"], training, tolerance,
                                          imu_topic=imu_topic)
                if (args.profile == MAVLINK_PROFILE
                        and result["selected_topic_inspection"][imu_topic]["rate_hz"] < contract["min_rate_hz"]):
                    raise CalibrationError(f"MAVLink held-out bag must actually contain >={contract['min_rate_hz']:g} Hz HIGHRES_IMU")
                result["result_sha256"] = sha256_file(chain_path)
                result.update(profile_annotations(args.profile))
                heldout_source = read_yaml(args.session_manifest) if args.session_manifest else {}
                errors = session_provenance_errors(heldout_source, result["heldout_bag_sha256"], "joint", args.profile)
                errors.extend(hardware_equivalence_errors(manifest, heldout_source))
                result.update(acquisition_manifest=heldout_source, formal_provenance_errors=errors,
                              formal_eligible=result["status"] == "PASSED" and not errors)
            except CalibrationError as error:
                result = {"status": "FAILED", "error": str(error), "result_sha256": sha256_file(chain_path)}
            write_yaml(validation / "heldout.yaml", result)
            report_html(validation / "report.html", "Held-out camera / IMU validation", result)
            if result["status"] != "PASSED":
                print(json.dumps(result, ensure_ascii=False))
                return 1
            write_yaml(validation_root / "selected.yaml", {"attempt": validation.name,
                "heldout_sha256": sha256_file(validation / "heldout.yaml"),
                "result_sha256": result["result_sha256"]})
        elif args.command == "prepare-openvins":
            result = {"path": str(prepare_openvins(args.candidate, args.estimator_template, args.profile)),
                      "status": "CANDIDATE_REPLAY_INPUT", **profile_annotations(args.profile)}
        else:
            result = {"path": str(publish(args.candidate, args.replay_evidence, args.destination, args.profile)),
                      "status": "VALIDATED", **profile_annotations(args.profile)}
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except (CalibrationError, FileExistsError, FileNotFoundError) as error:
        print(f"FAILED: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
