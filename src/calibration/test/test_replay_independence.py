"""Control metadata regressions; no sensor samples, bags, or acceptance files."""

from copy import deepcopy
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from spr_calibration.common import CalibrationError
from spr_calibration.export import _inspect_replay_streams, validate_replay_evidence
from spr_calibration.profiles import DEFAULT_PROFILE, MAVLINK_PROFILE
from spr_calibration.validation import validate_stream_independence


IMU_TOPIC = "/spr/calibration/imu"


def inspection_metadata():
    """Only fields consumed by report validation, not generated measurements."""
    topics = {}
    for index, topic in enumerate(("/left", "/right", IMU_TOPIC)):
        image = topic != IMU_TOPIC
        topics[topic] = {
            "type": "sensor_msgs/msg/Image" if image else "sensor_msgs/msg/Imu",
            "frame_id": f"camera_{index}_optical_frame" if image else "imu_link",
            "first_header_ns": 1_000_000_000,
            "last_header_ns": 2_000_000_000,
            "semantic_sha256": str(index + 1) * 64,
            "ordered_payload_sha256": str(index + 4) * 64,
            "rate_hz": 30.0 if image else 164.0,
        }
        if image:
            topics[topic]["profile"] = [848, 480, "mono8", 848]
    return topics


def independent_metadata(training):
    observed = deepcopy(training)
    for item in observed.values():
        item.update(first_header_ns=3_000_000_000, last_header_ns=4_000_000_000,
                    semantic_sha256="a" * 64, ordered_payload_sha256="b" * 64)
    return observed


class ReplayIndependenceTests(unittest.TestCase):
    def setUp(self):
        self.training = inspection_metadata()
        self.observed = independent_metadata(self.training)

    def check_streams(self, profile=MAVLINK_PROFILE):
        report = {"input_sha256": "independent-container", "topics": self.observed}
        with patch("spr_calibration.export.inspect_bag", return_value=report) as inspect:
            result = _inspect_replay_streams(Path("unopened-bag"), self.training, profile)
        inspect.assert_called_once_with(Path("unopened-bag"),
            {topic: item["type"] for topic, item in self.training.items()})
        return result

    def test_separate_independent_replay_and_different_valid_rates_are_allowed(self):
        self.observed[IMU_TOPIC]["rate_hz"] = 144.0
        self.observed["/left"]["rate_hz"] = 20.0
        self.assertEqual(self.check_streams()["topics"], self.observed)

    def test_repackaged_training_messages_are_rejected(self):
        self.observed["/left"]["semantic_sha256"] = self.training["/left"]["semantic_sha256"]
        with self.assertRaisesRegex(CalibrationError, "reuses training messages"):
            self.check_streams()

    def test_retimed_training_payloads_are_rejected(self):
        self.observed[IMU_TOPIC]["ordered_payload_sha256"] = self.training[IMU_TOPIC]["ordered_payload_sha256"]
        with self.assertRaisesRegex(CalibrationError, "identical training payloads"):
            self.check_streams()

    def test_overlapping_or_boundary_touching_header_ranges_are_rejected(self):
        self.observed["/right"]["first_header_ns"] = self.training["/right"]["last_header_ns"]
        with self.assertRaisesRegex(CalibrationError, "overlaps training header time range"):
            self.check_streams()

    def test_actual_replay_frame_and_image_profile_must_match(self):
        for key, value, message in (("frame_id", "other_camera", "frame differs"),
                                    ("profile", [640, 480, "mono8", 640], "image profile differs")):
            with self.subTest(key=key):
                self.observed = independent_metadata(self.training)
                self.observed["/left"][key] = value
                with self.assertRaisesRegex(CalibrationError, message):
                    self.check_streams()

    def test_matching_wrong_imu_frame_cannot_relabel_the_profile(self):
        self.training[IMU_TOPIC]["frame_id"] = "other_imu"
        self.observed[IMU_TOPIC]["frame_id"] = "other_imu"
        with self.assertRaisesRegex(CalibrationError, "IMU frame must be imu_link"):
            self.check_streams()

    def test_source_topic_set_must_match_explicit_profile(self):
        self.training["/spr/imu"] = self.training.pop(IMU_TOPIC)
        with self.assertRaisesRegex(CalibrationError, "two images and /spr/calibration/imu"):
            self.check_streams()

    def test_actual_bag_missing_expected_topic_is_rejected(self):
        with patch("spr_calibration.export.inspect_bag", side_effect=CalibrationError("Missing bag topics")):
            with self.assertRaisesRegex(CalibrationError, "Missing bag topics"):
                _inspect_replay_streams(Path("unopened-bag"), self.training, MAVLINK_PROFILE)

    def test_training_and_replay_must_both_meet_mavlink_minimum(self):
        for source in ("training", "observed"):
            with self.subTest(source=source):
                self.training = inspection_metadata()
                self.observed = independent_metadata(self.training)
                getattr(self, source)[IMU_TOPIC]["rate_hz"] = 139.9
                with self.assertRaisesRegex(CalibrationError, ">=140 Hz"):
                    self.check_streams()

    def test_dds_minimum_remains_200_hz(self):
        self.training["/spr/imu"] = self.training.pop(IMU_TOPIC)
        self.observed["/spr/imu"] = self.observed.pop(IMU_TOPIC)
        self.training["/spr/imu"]["rate_hz"] = 200.0
        self.observed["/spr/imu"]["rate_hz"] = 200.0
        self.check_streams(DEFAULT_PROFILE)
        self.observed["/spr/imu"]["rate_hz"] = 199.9
        with self.assertRaisesRegex(CalibrationError, ">=200 Hz"):
            self.check_streams(DEFAULT_PROFILE)

    def test_missing_fingerprint_or_range_cannot_disable_independence(self):
        for key in ("semantic_sha256", "ordered_payload_sha256", "first_header_ns"):
            with self.subTest(key=key):
                training = deepcopy(self.training)
                del training["/left"][key]
                with self.assertRaisesRegex(CalibrationError, "missing"):
                    validate_stream_independence(self.observed, training, IMU_TOPIC, label="Replay")

    def test_missing_measured_rate_is_rejected(self):
        del self.observed[IMU_TOPIC]["rate_hz"]
        with self.assertRaisesRegex(CalibrationError, "measured training/replay rate is missing"):
            self.check_streams()

    def test_public_validator_cannot_omit_training_inspection_or_profile(self):
        # Metadata exercises the control gate; no replay or evidence artifact is created.
        evidence = {"status": "PASSED", "config_sha256": "config", "bag_sha256": "different",
                    "run_id": "control-test", "diverged": False,
                    "reference": {"source": "sitl_gt", "accuracy": "control metadata"},
                    "metrics": {"ate_rmse_m": 0.0}}
        for arguments in ({"profile": MAVLINK_PROFILE}, {"training_topics": self.training}):
            with self.subTest(arguments=list(arguments)):
                with patch("spr_calibration.export._evidence_file") as artifact:
                    with self.assertRaisesRegex(CalibrationError, "training topic inspection.*explicitly selected"):
                        validate_replay_evidence(evidence, "config", "training", Path("unopened"), **arguments)
                    artifact.assert_not_called()

    def test_public_validator_inspects_actual_bag_and_rechecks_its_hash(self):
        evidence = {"status": "PASSED", "config_sha256": "config", "bag_sha256": "declared",
                    "bag_path": "unopened-bag", "run_id": "control-test", "diverged": False,
                    "reference": {"source": "sitl_gt", "accuracy": "control metadata"},
                    "metrics": {"ate_rmse_m": 0.0}}
        report = {"input_sha256": "changed", "topics": self.observed}
        # Mock external I/O only; the public validator must reach stream/hash validation.
        with patch("spr_calibration.export._evidence_file", return_value=Path("unopened-artifact")), \
                patch("spr_calibration.export.read_yaml", return_value={"argv": ["control-test"]}), \
                patch("spr_calibration.export.inspect_bag", return_value=report) as inspect:
            with self.assertRaisesRegex(CalibrationError, "bag does not match the evaluated input hash"):
                validate_replay_evidence(evidence, "config", "training", Path("unopened"),
                                         training_topics=self.training, profile=MAVLINK_PROFILE)
        inspect.assert_called_once_with(Path("unopened/unopened-bag"),
            {topic: item["type"] for topic, item in self.training.items()})


class RealInspectionMetadataTests(unittest.TestCase):
    def test_real_legacy_messages_remain_training_content_after_container_hash_change(self):
        fixture = Path(__file__).parent / "fixtures" / "legacy_mavros_inspection.json"
        training = json.loads(fixture.read_text(encoding="utf-8"))
        self.assertIn("/mavros/imu/data_raw", training["topics"])
        self.assertNotIn(IMU_TOPIC, training["topics"])
        self.assertIn("not MAVLink or formal", training["provenance"]["scope"])
        replay_metadata = deepcopy(training)
        # Container metadata only; no new bag or sensor stream is generated.
        replay_metadata["input_sha256"] = "0" * 64
        self.assertNotEqual(replay_metadata["input_sha256"], training["input_sha256"])
        with self.assertRaisesRegex(CalibrationError, "reuses training messages"):
            validate_stream_independence(replay_metadata["topics"], training["topics"],
                                         "/mavros/imu/data_raw", label="Replay")


if __name__ == "__main__":
    unittest.main()
