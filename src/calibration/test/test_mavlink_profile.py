"""Control/format regressions only; no generated calibration measurements."""

import unittest

from spr_calibration.cli import parser, topics_for
from spr_calibration.common import CalibrationError
from spr_calibration.profiles import (DEFAULT_PROFILE, MAVLINK_PROFILE,
                                      profile_annotations, profile_config, require_candidate_profile)


class MavlinkProfileTests(unittest.TestCase):
    def test_default_cli_keeps_dds_topic_contract(self):
        args = parser().parse_args(["inspect", "--bag", "recording"])
        self.assertEqual(args.profile, DEFAULT_PROFILE)
        self.assertIn("/spr/imu", topics_for(args))
        self.assertNotIn("/spr/calibration/imu", topics_for(args))

    def test_explicit_profile_selects_separate_mavlink_topic(self):
        args = parser().parse_args(["inspect", "--bag", "recording", "--profile", MAVLINK_PROFILE])
        self.assertIn("/spr/calibration/imu", topics_for(args))
        self.assertNotIn("/spr/imu", topics_for(args))
        self.assertEqual(profile_config(args.profile)["source"], "mavlink_highres_imu")

    def test_old_candidate_cannot_be_relabelled_mavlink(self):
        with self.assertRaisesRegex(CalibrationError, "dds_runtime"):
            require_candidate_profile({}, MAVLINK_PROFILE)
        with self.assertRaisesRegex(CalibrationError, "mavlink_calibration"):
            require_candidate_profile({"calibration_profile": MAVLINK_PROFILE}, DEFAULT_PROFILE)

    def test_mavlink_140hz_acceptance_cannot_claim_dds_fulfillment(self):
        annotations = profile_annotations(MAVLINK_PROFILE)
        self.assertEqual(annotations["minimum_observed_imu_rate_hz"], 140.0)
        self.assertEqual(annotations["deployment_scope"], "mavlink_140hz_test_only")
        self.assertIs(annotations["original_dds_200hz_requirement_satisfied"], False)
        self.assertEqual(require_candidate_profile(annotations, MAVLINK_PROFILE), MAVLINK_PROFILE)
        with self.assertRaisesRegex(CalibrationError, "deployment_scope"):
            require_candidate_profile(dict(annotations, deployment_scope="dds_runtime"), MAVLINK_PROFILE)

if __name__ == "__main__":
    unittest.main()
