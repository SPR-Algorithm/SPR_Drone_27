"""Held-out chessboard reprojection and inertial consistency validation.

The target MUST remain fixed while the rigid camera/IMU assembly moves.
These checks are independent of Kalibr fitting and never estimate replacement
camera/IMU extrinsics from the validation sequence.
"""

import cv2
import numpy as np

from .bags import header_ns, inspect_bag, load_imu, messages
from .common import CalibrationError, camera_chain, hash_input, transform


def board_points():
    points = np.zeros((8 * 11, 3), np.float64)
    points[:, :2] = np.mgrid[0:8, 0:11].T.reshape(-1, 2) * 0.025
    return points


def camera_matrix(camera):
    fx, fy, cx, cy = camera["intrinsics"]
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], float)


def image_array(message):
    if message.encoding not in ("mono8", "8UC1"):
        raise CalibrationError("Held-out validation requires uncompressed mono8 images")
    if len(message.data) != message.step * message.height or message.step < message.width:
        raise CalibrationError("Invalid image dimensions/step/data length")
    return np.asarray(message.data, dtype=np.uint8).reshape(message.height, message.step)[:, :message.width].copy()


def detect_board(image):
    found, corners = cv2.findChessboardCornersSB(image, (8, 11), flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        found, corners = cv2.findChessboardCorners(image, (8, 11),
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if found:
            corners = cv2.cornerSubPix(image, corners, (5, 5), (-1, -1),
                (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-4))
    return corners.reshape(-1, 2).astype(float) if found else None


def pose_from_corners(corners, camera):
    success, rotation, translation = cv2.solvePnP(board_points(), corners, camera_matrix(camera),
                                                 np.array(camera["distortion_coeffs"], float))
    if not success:
        raise CalibrationError("solvePnP failed for detected board")
    pose = np.eye(4)
    pose[:3, :3] = cv2.Rodrigues(rotation)[0]
    pose[:3, 3] = translation.ravel()
    return pose


def project(pose, camera):
    return cv2.projectPoints(board_points(), cv2.Rodrigues(pose[:3, :3])[0], pose[:3, 3],
                             camera_matrix(camera), np.array(camera["distortion_coeffs"], float))[0].reshape(-1, 2)


def rotation_velocities(times_s, rotations):
    times_s, rotations = np.asarray(times_s), np.asarray(rotations)
    dt = np.diff(times_s)
    if len(dt) < 4 or np.any(dt <= 0):
        raise CalibrationError("At least five monotonic visual poses required")
    # R_C_T maps a stationary target to a moving camera, hence the minus sign.
    omega = np.array([-cv2.Rodrigues(rotations[i + 1] @ rotations[i].T)[0].ravel() / dt[i]
                      for i in range(len(dt))])
    if np.any(np.linalg.norm(omega * dt[:, None], axis=1) > np.pi / 2):
        raise CalibrationError("Checkerboard pose flip (>90 degrees) detected; repeat without symmetric board flips")
    return (times_s[:-1] + times_s[1:]) / 2, omega


def check_time_sign(visual_times, visual_omega, imu_times, imu_gyro, rotation_cam_imu,
                    timeshift, search_radius_s=0.1, step_s=0.001):
    """Compare t_imu=t_cam+shift against an independent held-out gyro grid search."""
    visual_times, visual_omega = np.asarray(visual_times), np.asarray(visual_omega)
    imu_times, imu_gyro = np.asarray(imu_times), np.asarray(imu_gyro)
    valid = (visual_times - search_radius_s >= imu_times[0]) & (visual_times + search_radius_s <= imu_times[-1])
    valid &= np.linalg.norm(visual_omega, axis=1) < 8.0
    times, omega = visual_times[valid], visual_omega[valid]
    if len(times) < 20:
        raise CalibrationError("At least 20 held-out angular-rate comparisons with full time search overlap required")
    excitation = np.std(omega, axis=0)
    if np.any(excitation < 0.05):
        raise CalibrationError(f"Insufficient three-axis rotational excitation: {excitation}")

    def error(shift):
        interpolated = np.column_stack([np.interp(times + shift, imu_times, imu_gyro[:, axis]) for axis in range(3)])
        residual = interpolated @ np.asarray(rotation_cam_imu).T - omega
        bias = np.mean(residual, axis=0)
        return float(np.sqrt(np.mean(np.sum((residual - bias)**2, axis=1))))

    shifts = np.arange(-search_radius_s, search_radius_s + step_s / 2, step_s)
    errors = np.array([error(shift) for shift in shifts])
    best = int(np.argmin(errors))
    best_shift = float(shifts[best])
    shifted_error, opposite_error = error(timeshift), error(-timeshift)
    # An optimum on the search boundary is unobservable/outside the budget.
    edge = best in (0, len(shifts) - 1)
    offset_error = abs(best_shift - timeshift)
    passed = not edge and offset_error <= 0.005 and shifted_error <= 0.15
    # For a resolvable nonzero offset require the expected sign to improve fit.
    if abs(timeshift) > 0.005:
        passed = passed and shifted_error < opposite_error
    return {"status": "PASSED" if passed else "FAILED",
            "convention": "t_imu = t_cam + timeshift_cam_imu",
            "kalibr_timeshift_s": float(timeshift), "heldout_best_timeshift_s": best_shift,
            "offset_disagreement_s": float(offset_error), "maximum_disagreement_s": 0.005,
            "gyro_vector_rms_rad_s": shifted_error, "opposite_sign_rms_rad_s": opposite_error,
            "gyro_rms_limit_rad_s": 0.15, "boundary_optimum": edge,
            "axis_excitation_std_rad_s": excitation.tolist(), "comparisons": len(times),
            "search_timeshift_s": shifts.tolist(), "search_gyro_rms_rad_s": errors.tolist()}


def accelerometer_consistency(times, target_from_imu, imu_times, imu_accel, shift, window_s=0.4):
    """Validate the calibrated lever arm using visual position second derivatives.

    Fit only constant gravity in the target frame and a constant accel bias.
    A rank/conditioning gate rejects unobservable sequences. This is a noisy
    held-out physical consistency check, not a replacement calibration solver.
    """
    times = np.asarray(times)
    poses = np.asarray(target_from_imu)
    rows, rhs = [], []
    for index, time in enumerate(times):
        selected = np.flatnonzero(np.abs(times - time) <= window_s / 2)
        if len(selected) < 9 or selected[0] == index or selected[-1] == index:
            continue
        if time + shift < imu_times[0] or time + shift > imu_times[-1]:
            continue
        local = times[selected] - time
        design = np.column_stack([np.ones(len(local)), local, local**2, local**3])
        polynomial = np.linalg.lstsq(design, poses[selected, :3, 3], rcond=None)[0]
        acceleration_target = 2 * polynomial[2]
        rotation_imu_target = poses[index, :3, :3].T
        measured = np.array([np.interp(time + shift, imu_times, imu_accel[:, axis]) for axis in range(3)])
        rows.append(np.column_stack([rotation_imu_target, np.eye(3)]))
        rhs.append(measured - rotation_imu_target @ acceleration_target)
    if len(rows) < 30:
        raise CalibrationError("At least 30 well-supported held-out acceleration comparisons required")
    design, values = np.vstack(rows), np.concatenate(rhs)
    fit, _, rank, singular = np.linalg.lstsq(design, values, rcond=None)
    conditioning = float(singular[0] / singular[-1]) if singular[-1] > 0 else float("inf")
    residual = (design @ fit - values).reshape(-1, 3)
    rms = float(np.sqrt(np.mean(np.sum(residual**2, axis=1))))
    gravity, bias = float(np.linalg.norm(fit[:3])), float(np.linalg.norm(fit[3:]))
    passed = rank == 6 and conditioning < 1000 and 9.3 <= gravity <= 10.3 and bias < 1.0 and rms <= 0.7
    return {"status": "PASSED" if passed else "FAILED", "comparisons": len(rows),
            "specific_force_rms_m_s2": rms, "specific_force_rms_limit_m_s2": 0.7,
            "fitted_gravity_target_m_s2": (-fit[:3]).tolist(), "gravity_norm_m_s2": gravity,
            "gravity_convention": "specific_force_imu = R_imu_target * (position_ddot_target - gravity_target) + constant_bias_imu",
            "fitted_constant_accel_bias_m_s2": fit[3:].tolist(), "condition_number": conditioning,
            "rank": int(rank), "derivative_window_s": window_s,
            "note": "Checks rigid lever arm under fixed target assumption; no extrinsic refit"}


def validate_stream_independence(observed_topics, training_topics, imu_topic, label="Held-out"):
    """Compare actual bag inspection reports, including repackaged/retimed copies."""
    if (not isinstance(training_topics, dict) or not isinstance(observed_topics, dict)
            or not training_topics or set(observed_topics) != set(training_topics)):
        raise CalibrationError(f"Training/{label.lower()} topic fingerprints are missing or inconsistent")
    for topic, observed in observed_topics.items():
        training = training_topics[topic]
        if not isinstance(training, dict) or not isinstance(observed, dict):
            raise CalibrationError(f"{topic}: training/{label.lower()} inspection is malformed")
        if observed.get("type") != training.get("type"):
            raise CalibrationError(f"{topic}: {label.lower()} message type differs from training")
        if not observed.get("frame_id") or observed["frame_id"] != training.get("frame_id"):
            raise CalibrationError(f"{topic}: {label.lower()} frame differs from training")
        if topic == imu_topic and observed["frame_id"] != "imu_link":
            raise CalibrationError(f"{label} IMU frame must be imu_link")
        if observed.get("type") == "sensor_msgs/msg/Image" and (
                not observed.get("profile") or observed["profile"] != training.get("profile")):
            raise CalibrationError(f"{topic}: {label.lower()} image profile differs from training")
        for key in ("semantic_sha256", "ordered_payload_sha256"):
            if any(not isinstance(item.get(key), str) or len(item[key]) != 64
                   for item in (training, observed)):
                raise CalibrationError(f"{topic}: training/{label.lower()} {key} fingerprint is missing")
        if observed["semantic_sha256"] == training["semantic_sha256"]:
            raise CalibrationError(f"{topic}: {label.lower()} recording reuses training messages")
        if observed["ordered_payload_sha256"] == training["ordered_payload_sha256"]:
            raise CalibrationError(f"{topic}: {label.lower()} recording contains identical training payloads")
        for item in (training, observed):
            start, end = item.get("first_header_ns"), item.get("last_header_ns")
            if (any(isinstance(value, bool) or not isinstance(value, int) or value <= 0
                    for value in (start, end)) or end < start):
                raise CalibrationError(f"{topic}: training/{label.lower()} header time range is missing or invalid")
        if max(observed["first_header_ns"], training["first_header_ns"]) <= min(observed["last_header_ns"], training["last_header_ns"]):
            raise CalibrationError(f"{topic}: {label.lower()} recording overlaps training header time range")


def validate_heldout(bag, chain, training_hash, training_topics, pair_tolerance_s=0.001, max_pairs=None,
                     imu_topic="/spr/imu"):
    camera_chain(chain, joint=True)
    if hash_input(bag) == training_hash:
        raise CalibrationError("Held-out bag is identical to training input")
    left_topic, right_topic = (chain[name]["rostopic"] for name in ("cam0", "cam1"))
    topics = {left_topic: "sensor_msgs/msg/Image", right_topic: "sensor_msgs/msg/Image"}
    inspection = inspect_bag(bag, {**topics, imu_topic: "sensor_msgs/msg/Imu"})
    validate_stream_independence(inspection["topics"], training_topics, imu_topic)
    if not np.isfinite(pair_tolerance_s) or not 0 < pair_tolerance_s <= 0.01:
        raise CalibrationError("Recorded stereo pair tolerance must be positive and <=10 ms")
    pending = {left_topic: [], right_topic: []}
    pairs, detected, pixel_errors, pose_times, poses, previous_rotation = 0, 0, [], [], [], None
    stereo = transform(chain["cam1"]["T_cn_cnm1"], "T_c1_c0")
    imu_transform = transform(chain["cam0"]["T_cam_imu"], "T_c0_i")
    all_frames = {left_topic: set(), right_topic: set()}
    for topic, _, message in messages(bag, topics):
        expected = chain["cam0" if topic == left_topic else "cam1"]["resolution"]
        if [message.width, message.height] != expected:
            raise CalibrationError("Held-out image profile differs from calibration")
        stamp = header_ns(message)
        all_frames[topic].add(str(message.header.frame_id))
        pending[topic].append((stamp, message))
        while pending[left_topic] and pending[right_topic]:
            left_stamp, left_message = pending[left_topic][0]
            right_stamp, right_message = pending[right_topic][0]
            if abs(left_stamp - right_stamp) > pair_tolerance_s * 1e9:
                pending[left_topic if left_stamp < right_stamp else right_topic].pop(0)
                continue
            pending[left_topic].pop(0)
            pending[right_topic].pop(0)
            pairs += 1
            corners_left = detect_board(image_array(left_message))
            corners_right = detect_board(image_array(right_message))
            if corners_left is None or corners_right is None:
                continue
            # Both checkerboard orderings describe the same physical points.
            # Keep the left ordering continuous; select right ordering using the
            # frozen stereo transform, never estimate a new stereo transform.
            choices = []
            for candidate in (corners_left, corners_left[::-1]):
                pose = pose_from_corners(candidate, chain["cam0"])
                distance = 0 if previous_rotation is None else np.linalg.norm(
                    cv2.Rodrigues(pose[:3, :3] @ previous_rotation.T)[0])
                choices.append((distance, pose, candidate))
            _, pose, ordered = min(choices, key=lambda item: item[0])
            predicted = project(stereo @ pose, chain["cam1"])
            right = min((corners_right, corners_right[::-1]), key=lambda value: np.mean((predicted - value)**2))
            errors = np.linalg.norm(predicted - right, axis=1)
            left_errors = np.linalg.norm(project(pose, chain["cam0"]) - ordered, axis=1)
            pixel_errors.append(np.concatenate([left_errors, errors]))
            previous_rotation = pose[:3, :3]
            pose_times.append(left_stamp)
            poses.append(pose)
            detected += 1
            if max_pairs is not None and detected >= max_pairs:
                break
    if detected < 30:
        raise CalibrationError(f"Held-out validation found {detected} board pairs; at least 30 required")
    first_ns = int(pose_times[0])
    times = (np.asarray(pose_times, np.int64) - first_ns) / 1e9
    poses = np.asarray(poses)
    if np.any(np.diff(times) <= 0):
        raise CalibrationError("Held-out image stamps repeat or regress")
    visual_times, omega = rotation_velocities(times, poses[:, :3, :3])
    imu_ns, imu_values = load_imu(bag, imu_topic)
    imu_times = (imu_ns - first_ns) / 1e9
    shift = float(chain["cam0"]["timeshift_cam_imu"])
    timing = check_time_sign(visual_times, omega, imu_times, imu_values[:, :3], imu_transform[:3, :3], shift)
    target_from_imu = np.asarray([np.linalg.inv(pose) @ imu_transform for pose in poses])
    acceleration = accelerometer_consistency(times, target_from_imu, imu_times, imu_values[:, 3:], shift)
    errors = np.asarray(pixel_errors)
    rms = float(np.sqrt(np.mean(errors**2)))
    per_view = np.sqrt(np.mean(errors**2, axis=1))
    pair_ratio = pairs / max(inspection["topics"][left_topic]["count"], inspection["topics"][right_topic]["count"])
    passed = (rms <= 0.5 and float(np.max(per_view)) <= 1.0 and pair_ratio >= 0.95
              and timing["status"] == "PASSED" and acceleration["status"] == "PASSED")
    return {"status": "PASSED" if passed else "FAILED", "heldout_bag_sha256": hash_input(bag),
            "training_bag_sha256": training_hash, "detected_pairs": detected, "paired_images": pairs,
            "selected_topic_inspection": inspection["topics"], "pair_tolerance_s": pair_tolerance_s,
            "stereo_pair_ratio": pair_ratio, "minimum_pair_ratio": 0.95,
            "reprojection_rms_px": rms, "reprojection_limit_px": 0.5,
            "max_view_rms_px": float(np.max(per_view)), "max_view_limit_px": 1.0,
            "time_sign": timing, "accelerometer": acceleration,
            "reference": {"source": "fixed_checkerboard", "accuracy": "25 mm pitch; printed target must be measured"},
            "visual_pose_note": "PnP estimates each target pose; frozen stereo and IMU transforms are tested on held-out measurements",
            "frames": {key: sorted(value) for key, value in all_frames.items()}}
