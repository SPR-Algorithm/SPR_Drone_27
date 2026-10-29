"""Format/contract regressions. Hardware acceptance requires real recorded data."""

import os
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from spr_calibration.common import CalibrationError, imu_config, read_yaml, target_config, transform
from spr_calibration.bags import convert_bag
from spr_calibration.export import _opencv_yaml, validate_replay_evidence
from spr_calibration.solver import provenance_errors


class JointContractTests(unittest.TestCase):
    def test_board_is_inner_corners_and_meters(self):
        self.assertEqual(target_config(), {"target_type": "checkerboard", "targetCols": 8,
                         "targetRows": 11, "rowSpacingMeters": 0.025, "colSpacingMeters": 0.025})

    def test_improper_rotation_is_rejected(self):
        reflection = np.eye(4)
        reflection[0, 0] = -1
        with self.assertRaisesRegex(CalibrationError, "SO"):
            transform(reflection, "reflection")

    def test_mavros_and_realsense_imu_cannot_be_formal_sources(self):
        for topic in ("/mavros/imu/data_raw", "/camera/imu"):
            noise = {key: 1.0 for key in ("accelerometer_noise_density", "accelerometer_random_walk",
                     "gyroscope_noise_density", "gyroscope_random_walk", "update_rate")}
            noise["rostopic"] = topic
            with self.assertRaisesRegex(CalibrationError, "/spr/imu"):
                imu_config(noise)

    def test_absent_provenance_stays_unverified(self):
        errors = provenance_errors({})
        self.assertTrue(any("bridge_commit" in message for message in errors))
        self.assertTrue(any("timesync_snapshot" in message for message in errors))
        self.assertTrue(any("exposure" in message for message in errors))

    def test_opencv_dialect_is_only_generated_at_final_export(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "final.yaml"
            _opencv_yaml(path, {"max_cameras": 2, "cam0": {"T_cam_imu": np.eye(4).tolist(),
                                                         "intrinsics": [400.0, 400.0, 424.0, 240.0]}})
            storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
            self.assertTrue(storage.isOpened())
            self.assertEqual(storage.getNode("max_cameras").real(), 2)
            storage.release()
            with self.assertRaisesRegex(CalibrationError, "OpenCV YAML"):
                read_yaml(path)

    def test_no_groundtruth_does_not_allow_ate_claim(self):
        evidence = {"status": "PASSED", "config_sha256": "config", "bag_sha256": "heldout",
                    "run_id": "replay", "diverged": False, "reference": {"source": "none"},
                    "metrics": {"ate_rmse_m": 0.01}}
        with self.assertRaisesRegex(CalibrationError, "ATE/RPE"):
            validate_replay_evidence(evidence, "config", "training")

    def test_replay_is_bound_to_exact_export_and_heldout_bag(self):
        evidence = {"status": "PASSED", "config_sha256": "wrong", "bag_sha256": "heldout"}
        with self.assertRaisesRegex(CalibrationError, "exact OpenVINS"):
            validate_replay_evidence(evidence, "config", "training")


@unittest.skipUnless(os.environ.get("SPR_CALIBRATION_REAL_BAG"),
                     "Set SPR_CALIBRATION_REAL_BAG to an actual stopped rosbag2 recording")
class RealBagIntegrationTests(unittest.TestCase):
    def test_all_ros2_messages_survive_ros1_conversion(self):
        source = Path(os.environ["SPR_CALIBRATION_REAL_BAG"])
        topics = {"/camera/infra1/image_rect_raw": "sensor_msgs/msg/Image",
                  "/camera/infra2/image_rect_raw": "sensor_msgs/msg/Image",
                  "/spr/imu": "sensor_msgs/msg/Imu"}
        with tempfile.TemporaryDirectory() as directory:
            result = convert_bag(source, Path(directory) / "input.bag", topics)
            self.assertTrue(result["verified_all_messages"])
            self.assertEqual(result["source"]["topics"], result["converted"]["topics"])

if __name__ == "__main__":
    unittest.main()
