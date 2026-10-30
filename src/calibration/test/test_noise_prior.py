"""Noise-model metadata/control regressions; no sensor datasets are generated."""

from contextlib import redirect_stderr
from copy import deepcopy
import io
from pathlib import Path
import unittest
from unittest.mock import patch

from spr_calibration.cli import main, parser
from spr_calibration.common import CalibrationError
from spr_calibration.noise import NOISE_PRIOR_UNITS, validate_noise_prior
from spr_calibration.profiles import DEFAULT_PROFILE, MAVLINK_PROFILE
from spr_calibration.solver import solve


def model_metadata():
    """Explicit control values, not measured noise or an operational prior."""
    return {"rostopic": "/spr/calibration/imu", "update_rate": 160.0,
            "accelerometer_noise_density": 0.002, "accelerometer_random_walk": 0.0004,
            "gyroscope_noise_density": 0.0003, "gyroscope_random_walk": 0.00001}


def prior_metadata(imu):
    return {"schema_version": 1, "calibration_profile": MAVLINK_PROFILE,
            "basis": "engineering_prior", "noise_model_measured": False,
            "original_allan_requirement_satisfied": False, "imu": deepcopy(imu),
            "parameter_sources": {
                key: {"kind": "engineering_assumption", "reference": "unit-test control metadata",
                      "rationale": "Exercise explicit attribution without estimating noise", "unit": unit}
                for key, unit in NOISE_PRIOR_UNITS.items()}}


def solve_arguments():
    return ["solve", "--bag", "unopened-bag", "--camchain", "unopened-cameras.yaml",
            "--imu", "imu.yaml", "--output", "uncreated-output", "--profile", MAVLINK_PROFILE]


class NoisePriorTests(unittest.TestCase):
    def setUp(self):
        self.imu = model_metadata()
        self.document = prior_metadata(self.imu)

    def validate(self):
        return validate_noise_prior(self.document, self.imu, MAVLINK_PROFILE)

    def test_explicit_prior_preserves_every_supplied_model_value(self):
        original_model, original_document = deepcopy(self.imu), deepcopy(self.document)
        self.assertEqual(self.validate(), {"noise_basis": "engineering_prior", "noise_model_measured": False,
                                          "original_allan_requirement_satisfied": False})
        self.assertEqual(self.imu, original_model)
        self.assertEqual(self.document, original_document)
        self.assertNotIn("deployment_scope", self.validate())

    def test_dds_cannot_use_an_engineering_prior(self):
        with self.assertRaisesRegex(CalibrationError, "only for mavlink_calibration"):
            validate_noise_prior(self.document, self.imu, DEFAULT_PROFILE)

    def test_datasheet_attribution_still_describes_an_unmeasured_model(self):
        self.document["parameter_sources"]["gyroscope_noise_density"]["kind"] = "datasheet_prior"
        annotations = self.validate()
        self.assertIs(annotations["noise_model_measured"], False)
        self.assertIs(annotations["original_allan_requirement_satisfied"], False)

    def test_schema_and_profile_must_be_explicit(self):
        for key, value in (("schema_version", True), ("schema_version", 2),
                           ("calibration_profile", DEFAULT_PROFILE), ("basis", "measured_allan")):
            with self.subTest(key=key, value=value):
                self.document = prior_metadata(self.imu)
                self.document[key] = value
                with self.assertRaises(CalibrationError):
                    self.validate()

    def test_prior_cannot_claim_measurement_or_the_original_allan_requirement(self):
        for key in ("noise_model_measured", "original_allan_requirement_satisfied"):
            for value in (True, 0, None):
                with self.subTest(key=key, value=value):
                    self.document = prior_metadata(self.imu)
                    self.document[key] = value
                    with self.assertRaisesRegex(CalibrationError, key + ": false"):
                        self.validate()

    def test_each_noise_coefficient_requires_a_source(self):
        for key in NOISE_PRIOR_UNITS:
            with self.subTest(key=key):
                self.document = prior_metadata(self.imu)
                del self.document["parameter_sources"][key]
                with self.assertRaisesRegex(CalibrationError, key + " is missing"):
                    self.validate()

    def test_source_fields_cannot_be_empty_or_claim_measured_noise(self):
        key = "gyroscope_random_walk"
        for field, value in (("kind", "measured"), ("reference", " "), ("rationale", ""), ("unit", None)):
            with self.subTest(field=field):
                self.document = prior_metadata(self.imu)
                self.document["parameter_sources"][key][field] = value
                with self.assertRaisesRegex(CalibrationError, key + "." + field):
                    self.validate()

    def test_each_coefficient_uses_its_kalibr_continuous_time_unit(self):
        for key in NOISE_PRIOR_UNITS:
            with self.subTest(key=key):
                self.document = prior_metadata(self.imu)
                self.document["parameter_sources"][key]["unit"] = "per_sample"
                with self.assertRaisesRegex(CalibrationError, key + ".unit must be"):
                    self.validate()

    def test_prior_mapping_must_match_values_rate_topic_and_all_keys(self):
        for key, value in (("accelerometer_noise_density", 0.003), ("update_rate", 144.0),
                           ("rostopic", "/spr/imu"), ("unmatched_key", "metadata")):
            with self.subTest(key=key):
                self.document = prior_metadata(self.imu)
                self.document["imu"][key] = value
                with self.assertRaises(CalibrationError):
                    self.validate()

    def test_prior_and_model_both_require_positive_finite_coefficients(self):
        for key in (*NOISE_PRIOR_UNITS, "update_rate"):
            for value in (0.0, -1.0, float("nan"), True):
                with self.subTest(key=key, value=value):
                    self.imu = model_metadata()
                    self.imu[key] = value
                    self.document = prior_metadata(self.imu)
                    with self.assertRaisesRegex(CalibrationError, "positive and finite"):
                        self.validate()

    def test_boolean_cannot_match_a_numeric_prior_coefficient(self):
        self.imu["gyroscope_random_walk"] = 1.0
        self.document = prior_metadata(self.imu)
        self.document["imu"]["gyroscope_random_walk"] = True
        with self.assertRaisesRegex(CalibrationError, "positive and finite"):
            self.validate()

    def test_cli_prior_and_allan_options_are_mutually_exclusive(self):
        with redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as caught:
                parser().parse_args(solve_arguments() + ["--noise-prior", "prior.yaml", "--allan-report", "allan.yaml"])
        self.assertEqual(caught.exception.code, 2)

    def test_cli_rejects_invalid_prior_before_calling_solver(self):
        self.document["parameter_sources"]["gyroscope_random_walk"]["unit"] = "per_sample"
        documents = {Path("prior.yaml"): self.document, Path("imu.yaml"): self.imu}
        with patch("spr_calibration.cli.read_yaml", side_effect=documents.__getitem__), \
                patch("spr_calibration.cli.solve") as execute, redirect_stderr(io.StringIO()):
            self.assertEqual(main(solve_arguments() + ["--noise-prior", "prior.yaml"]), 2)
        execute.assert_not_called()

    def test_cli_passes_validated_prior_as_an_explicit_solver_keyword(self):
        documents = {Path("prior.yaml"): self.document, Path("imu.yaml"): self.imu}
        with patch("spr_calibration.cli.read_yaml", side_effect=documents.__getitem__), \
                patch("spr_calibration.cli.solve", side_effect=CalibrationError("stop before real solver")) as execute, \
                redirect_stderr(io.StringIO()):
            self.assertEqual(main(solve_arguments() + ["--noise-prior", "prior.yaml"]), 2)
        self.assertEqual(execute.call_args.kwargs, {"noise_prior": Path("prior.yaml")})

    def test_direct_solver_rejects_prior_before_hashing_bag_or_running_docker(self):
        self.document["noise_model_measured"] = True
        documents = {Path("prior.yaml"): self.document, Path("imu.yaml"): self.imu}
        with patch("spr_calibration.solver.read_yaml", side_effect=documents.__getitem__), \
                patch("spr_calibration.solver.hash_input") as bag_hash, \
                patch("spr_calibration.solver.subprocess.run") as process:
            with self.assertRaisesRegex(CalibrationError, "noise_model_measured: false"):
                solve(Path("unopened-bag"), Path("unopened-cameras.yaml"), Path("imu.yaml"),
                      Path("uncreated-output"), profile=MAVLINK_PROFILE, noise_prior=Path("prior.yaml"))
        bag_hash.assert_not_called()
        process.assert_not_called()


if __name__ == "__main__":
    unittest.main()
