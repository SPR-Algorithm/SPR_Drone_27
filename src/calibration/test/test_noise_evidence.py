"""Publication-control regressions; these declarations are not sensor captures."""

from copy import deepcopy
from pathlib import Path
import tempfile
import unittest

from spr_calibration.common import CalibrationError, sha256_file, write_yaml
from spr_calibration.export import validate_candidate_noise
from spr_calibration.noise import NOISE_PRIOR_UNITS
from spr_calibration.profiles import DEFAULT_PROFILE, MAVLINK_PROFILE


class NoiseEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name)
        self.imu = {key: 0.01 for key in NOISE_PRIOR_UNITS}
        self.imu.update(rostopic="/spr/calibration/imu", update_rate=164.0)
        self.prior = {
            "schema_version": 1, "basis": "engineering_prior",
            "calibration_profile": MAVLINK_PROFILE,
            "noise_model_measured": False, "original_allan_requirement_satisfied": False,
            "imu": self.imu,
            "parameter_sources": {key: {
                "kind": "engineering_assumption", "reference": "control-test declaration",
                "rationale": "Exercises evidence integrity; no physical measurement claim", "unit": unit,
            } for key, unit in NOISE_PRIOR_UNITS.items()},
        }
        write_yaml(self.path / "imu.yaml", self.imu)
        write_yaml(self.path / "noise_prior.yaml", self.prior)
        self.manifest = {
            "noise_basis": "engineering_prior", "noise_model_measured": False,
            "original_allan_requirement_satisfied": False,
            "artifact_sha256": {"noise_prior.yaml": sha256_file(self.path / "noise_prior.yaml")},
        }

    def test_mavlink_accepts_bound_prior_without_claiming_allan(self):
        filename, annotations = validate_candidate_noise(self.path, self.manifest, MAVLINK_PROFILE)
        self.assertEqual(filename, "noise_prior.yaml")
        self.assertIs(annotations["noise_model_measured"], False)
        self.assertIs(annotations["original_allan_requirement_satisfied"], False)
        self.assertFalse((self.path / "allan_report.yaml").exists())

    def test_dds_cannot_use_prior_exception(self):
        with self.assertRaises(CalibrationError):
            validate_candidate_noise(self.path, self.manifest, DEFAULT_PROFILE)

    def test_prior_change_after_solve_is_rejected(self):
        changed = deepcopy(self.prior)
        changed["parameter_sources"]["gyroscope_random_walk"]["rationale"] = "Changed after fitting"
        write_yaml(self.path / "noise_prior.yaml", changed)
        with self.assertRaisesRegex(CalibrationError, "missing or changed"):
            validate_candidate_noise(self.path, self.manifest, MAVLINK_PROFILE)

    def test_noise_values_cannot_change_after_solve(self):
        self.imu["gyroscope_random_walk"] *= 2
        write_yaml(self.path / "imu.yaml", self.imu)
        with self.assertRaisesRegex(CalibrationError, "exactly match"):
            validate_candidate_noise(self.path, self.manifest, MAVLINK_PROFILE)

    def test_prior_hash_is_required(self):
        self.manifest["artifact_sha256"] = {}
        with self.assertRaisesRegex(CalibrationError, "missing or changed"):
            validate_candidate_noise(self.path, self.manifest, MAVLINK_PROFILE)

    def test_published_scope_cannot_claim_measured_or_omit_flags(self):
        for key in ("noise_model_measured", "original_allan_requirement_satisfied"):
            for value in (None, True, 0):
                manifest = deepcopy(self.manifest)
                manifest[key] = value
                with self.subTest(key=key, value=value), self.assertRaisesRegex(CalibrationError, "preserve"):
                    validate_candidate_noise(self.path, manifest, MAVLINK_PROFILE)


if __name__ == "__main__":
    unittest.main()
