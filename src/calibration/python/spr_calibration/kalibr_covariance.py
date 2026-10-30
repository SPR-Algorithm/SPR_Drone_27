"""Standalone pinned-Kalibr analytic marginal covariance adapter (Python 3.8+).

Kalibr and the native selected-column bridge are imported only during execution.
The optimizer state and its error terms are retained without another optimization.
"""

import importlib
import operator
from pathlib import Path
import runpy

import numpy as np
import yaml


KALIBR_MODULE = "kalibr_imu_camera_calibration.IccCalibrator"
KALIBR_ENTRY = "/catkin_ws/src/kalibr/aslam_offline_calibration/kalibr/python/kalibr_calibrate_imu_camera"
ORDERING = ["rotation_x_rad", "rotation_y_rad", "rotation_z_rad",
            "translation_x_m", "translation_y_m", "translation_z_m",
            "cam0_timeshift_s", "cam1_timeshift_s"]


def calibration_blocks(camera_chain, hessian):
    """Read actual active block indices in the report's physical parameter order."""
    if len(camera_chain.camList) != 2:
        raise RuntimeError("Joint covariance requires exactly two cameras")
    cam0, cam1 = camera_chain.camList
    # Pinned aslam_backend.TransformationDv.getDesignVariable: 0=q, 1=t.
    selected = [
        ("cam0_rotation", cam0.T_c_b_Dv.getDesignVariable(0), 3),
        ("cam0_translation", cam0.T_c_b_Dv.getDesignVariable(1), 3),
        ("cam0_timeshift", cam0.cameraTimeToImuTimeDv, 1),
        ("cam1_timeshift", cam1.cameraTimeToImuTimeDv, 1),
    ]
    blocks = []
    for name, variable, expected_dimension in selected:
        if not variable.isActive():
            raise RuntimeError("Covariance design variable is inactive: " + name)
        try:
            raw_index = variable.blockIndex()
            if isinstance(raw_index, bool):
                raise TypeError("boolean block index")
            index = operator.index(raw_index)
        except TypeError as error:
            raise RuntimeError("Covariance block index must be an integer: " + name) from error
        dimension = int(variable.minimalDimensions())
        if index < 0 or index >= hessian.bRows() or index >= hessian.bCols():
            raise RuntimeError("Covariance block index is outside the full Hessian: " + name)
        if (dimension != expected_dimension or hessian.rowsOfBlock(index) != dimension
                or hessian.colsOfBlock(index) != dimension):
            raise RuntimeError("Covariance minimal/block dimension mismatch: " + name)
        # The pinned calibration leaves these scalings at their unit defaults.
        # Reject other coordinates instead of silently changing covariance units.
        if variable.scaling() != 1.0:
            raise RuntimeError("Covariance requires unit design-variable scaling: " + name)
        blocks.append({"parameter": name, "block_index": int(index),
                       "minimal_dimension": dimension,
                       "scalar_base": int(hessian.rowBaseOfBlock(index))})
    if len({block["block_index"] for block in blocks}) != len(blocks):
        raise RuntimeError("Covariance design variables have duplicate block indices")
    return blocks


def validate_covariance(covariance):
    """Validate the selected marginal without clipping, symmetrizing or inversion."""
    if np.iscomplexobj(covariance):
        raise RuntimeError("Joint covariance must be real")
    covariance = np.asarray(covariance, dtype=float)
    if covariance.shape != (8, 8) or not np.isfinite(covariance).all():
        raise RuntimeError("Joint covariance must be a finite 8x8 matrix")
    if not np.allclose(covariance, covariance.T, rtol=1e-8, atol=1e-12):
        raise RuntimeError("Joint covariance is not symmetric")
    try:
        np.linalg.cholesky(covariance)
    except np.linalg.LinAlgError as error:
        raise RuntimeError("Joint covariance is not positive definite") from error
    return covariance


def analytic_marginal_covariance(calibrator, backend, marginal_covariance):
    """Build full analytic H at the converged state, then select blocks of H^-1."""
    options = backend.Optimizer2Options()
    options.linearSolver = backend.BlockCholeskyLinearSystemSolver()
    options.doSchurComplement = False
    options.nThreads = 1
    options.verbose = True
    optimizer = backend.Optimizer2(options)
    optimizer.setProblem(calibrator.problem)
    optimizer.initialize()
    hessian = backend.SparseBlockMatrixXd()
    # Pinned computeHessian calls every term's analytic buildHessian, including
    # BSplineMotionError's Q contribution; it never requests its generic Jacobian.
    optimizer.computeHessian(hessian, 0.0)
    rows, cols = int(hessian.rows()), int(hessian.cols())
    if rows != cols or rows < len(ORDERING):
        raise RuntimeError("Full analytic Hessian must be square and contain all selected parameters")
    blocks = calibration_blocks(calibrator.CameraChain, hessian)
    indices = [block["block_index"] for block in blocks]
    # The native bridge factors FULL H once and solves its selected unit columns. Inverting only
    # H_cc would condition on nuisance variables instead of marginalizing them.
    covariance = validate_covariance(marginal_covariance(hessian, indices))
    metadata = {
        "schema_version": 1,
        "method": "full_analytic_hessian_cholmod_selected_columns",
        "ordering": list(ORDERING),
        "selected_blocks": blocks,
        "selected_block_indices": indices,
        "selected_minimal_dimensions": [block["minimal_dimension"] for block in blocks],
        "full_hessian_dimensions": {"rows": rows, "cols": cols,
                                    "block_rows": int(hessian.bRows()), "block_cols": int(hessian.bCols())},
        "diagonal_damping": 0.0,
        "optimizer_rerun": False,
        "all_problem_error_terms_retained": True,
        "bias_spline_analytic_Q_retained": True,
        "robust_loss_weighting": False,
        "full_nuisance_variables_marginalized": True,
        "selected_inverse": "(H^-1)[selected,selected], not inverse(H[selected,selected])",
        "conditional_on_configured_noise_model": True,
        "noise_model_parameter_uncertainty_marginalized": False,
        "interpretation": "Local linearized covariance conditional on supplied fixed IMU noise coefficients; "
                          "engineering prior coefficient uncertainty and systematic errors are not propagated",
        "validation": {"finite": True, "positive_definite": True,
                       "symmetry_rtol": 1e-8, "symmetry_atol": 1e-12,
                       "clipping": False, "pseudoinverse": False},
    }
    return covariance, metadata


def recover_covariance(calibrator):
    output = Path("/data")
    # This is a diagnostic snapshot, never the completed candidate result.
    calibrator.saveCamChainParametersYaml(str(output / "pre_covariance_camchain.yaml"))
    module = importlib.import_module(KALIBR_MODULE)
    extension = importlib.import_module("spr_kalibr_covariance")
    print("Recovering full analytic marginal covariance (zero damping)...", flush=True)
    covariance, metadata = analytic_marginal_covariance(calibrator, module.aopt, extension.marginal_covariance)
    np.save(str(output / "joint_covariance.npy"), covariance)
    with (output / "covariance_method.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(metadata, stream, sort_keys=False)
    deviations = np.sqrt(np.diag(covariance))
    calibrator.std_trafo_ic = deviations[:6].copy()
    calibrator.std_times = deviations[6:].copy()


def main():
    module = importlib.import_module(KALIBR_MODULE)
    module.IccCalibrator.recoverCovariance = recover_covariance
    runpy.run_path(KALIBR_ENTRY, run_name="__main__")


if __name__ == "__main__":
    main()
