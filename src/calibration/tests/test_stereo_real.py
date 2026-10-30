"""Replay an actual, qualified D435i capture; never create image fixtures."""

import hashlib
import os
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np
import yaml


def capture_hashes(session):
    paths = [session / "meta.yaml", session / "index.csv"]
    paths += sorted((session / "left").glob("*.png"))
    paths += sorted((session / "right").glob("*.png"))
    assert len(paths) > 2, f"No captured PNG pairs in {session}"
    return {str(path.relative_to(session)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in paths}


def main():
    selected = os.environ.get("SPR_CALIBRATION_REAL_SESSION")
    if not selected:
        print("SKIP: set SPR_CALIBRATION_REAL_SESSION to a real, stopped D435i "
              "capture with >=30 pairs and >=80% coverage in both cameras")
        return 77

    session = Path(selected).resolve(strict=True)
    before = capture_hashes(session)
    with tempfile.TemporaryDirectory(prefix="spr-real-stereo-") as directory:
        output = Path(directory) / "calibration"
        result = subprocess.run(
            [sys.argv[1], "--session", str(session), "--output", str(output)],
            capture_output=True, text=True, timeout=280,
        )
        print(result.stdout + result.stderr, end="")
        assert result.returncode == 0, "Real capture did not pass default calibration gates"
        report = yaml.safe_load((output / "report.yaml").read_text())
        chain = yaml.safe_load((output / "camchain.yaml").read_text())
        assert report["success"] and report["formal_eligible"] and not report["diagnostic"]
        assert (report["board_cols"], report["board_rows"]) == (8, 11)
        assert np.isclose(report["square_size_m"], 0.025)
        assert report["input_views"] >= 30 and report["holdout_views"] > 0
        assert report["stereo_rms"] <= 0.5 and report["holdout_rms_px"] <= 0.5
        assert report["holdout_max_view_rms_px"] <= 1.0
        assert min(report["coverage_left"], report["coverage_right"]) >= 0.8

        # Export must preserve the reported cam0 -> cam1 transform, in meters.
        transform = np.asarray(chain["cam1"]["T_cn_cnm1"], dtype=float)
        np.testing.assert_allclose(transform[:3, :3], report["R_cam1_cam0"], atol=1e-12)
        np.testing.assert_allclose(transform[:3, 3],
                                   np.asarray(report["T_cam1_cam0"]).reshape(3), atol=1e-12)
        np.testing.assert_allclose(transform[3], [0, 0, 0, 1], atol=1e-12)
        rotation = transform[:3, :3]
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-8)
        assert np.isclose(np.linalg.det(rotation), 1.0)
        assert transform[0, 3] < 0, "D435i cam0 -> cam1 baseline direction reversed"
        assert np.isclose(np.linalg.norm(transform[:3, 3]), report["baseline_m"])
        assert "T_cam_imu" not in chain["cam0"] and "T_cam_imu" not in chain["cam1"]
        for name in ("cam0", "cam1"):
            assert (output / f"{name}.yaml").is_file()
            assert np.isfinite(chain[name]["intrinsics"]).all()

        covariance = np.asarray(report["stereo_global_covariance"], dtype=float)
        assert report["stereo_uncertainty_available"] and covariance.shape == (22, 22)
        assert np.isfinite(covariance).all() and np.all(np.diag(covariance) > 0)
        assert capture_hashes(session) == before, "Regression changed captured inputs"
        print(f"PASS: real D435i capture; {report['input_views']} detected pairs, "
              f"RMS {report['stereo_rms']:.6f} px, "
              f"held-out RMS {report['holdout_rms_px']:.6f} px")
    return 0


if __name__ == "__main__":
    sys.exit(main())
