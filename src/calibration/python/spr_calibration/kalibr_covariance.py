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


def _structure_integer(value, label):
    try:
        if isinstance(value, bool):
            raise TypeError("boolean dimension/index")
        return int(operator.index(value))
    except TypeError as error:
        raise RuntimeError("Hessian structure requires an integer " + label) from error


def _support_structure(hessian, blocks, structure):
    """Check native exact-zero bookkeeping without weakening its coefficient test."""
    if not isinstance(structure, dict):
        raise RuntimeError("Native Hessian structure must be a mapping")
    rows, cols = int(hessian.rows()), int(hessian.cols())
    dimensions = structure.get("full_dimensions")
    if (not isinstance(dimensions, (list, tuple)) or len(dimensions) != 2
            or [_structure_integer(value, "full dimension") for value in dimensions] != [rows, cols]):
        raise RuntimeError("Native Hessian structure dimensions differ from the full Hessian")
    zero_blocks, zero_rows = structure.get("exact_zero_blocks"), structure.get("exact_zero_scalar_rows")
    if not isinstance(zero_blocks, list) or not isinstance(zero_rows, list):
        raise RuntimeError("Native Hessian structure must list exact-zero blocks and scalar rows")
    zero_rows = [_structure_integer(value, "zero scalar row") for value in zero_rows]
    if len(set(zero_rows)) != len(zero_rows) or any(value < 0 or value >= rows for value in zero_rows):
        raise RuntimeError("Native exact-zero scalar rows are duplicated or outside the full Hessian")
    checked_blocks, block_indices, whole_zero_rows = [], set(), set()
    for block in zero_blocks:
        if not isinstance(block, dict):
            raise RuntimeError("Native exact-zero block must be a mapping")
        index = _structure_integer(block.get("block_index"), "zero block index")
        base = _structure_integer(block.get("scalar_base"), "zero block scalar base")
        dimension = _structure_integer(block.get("dimension"), "zero block dimension")
        if (index < 0 or index >= hessian.bRows() or index >= hessian.bCols() or index in block_indices
                or base != hessian.rowBaseOfBlock(index) or dimension <= 0
                or dimension != hessian.rowsOfBlock(index) or dimension != hessian.colsOfBlock(index)):
            raise RuntimeError("Native exact-zero block does not match the full Hessian layout")
        scalar_rows = set(range(base, base + dimension))
        if not scalar_rows.issubset(zero_rows):
            raise RuntimeError("Native exact-zero block is not entirely covered by exact-zero scalar rows")
        block_indices.add(index)
        whole_zero_rows.update(scalar_rows)
        checked_blocks.append({"block_index": index, "scalar_base": base, "dimension": dimension})
    support_dimension = _structure_integer(structure.get("support_scalar_dimension"), "support dimension")
    if support_dimension != rows - len(whole_zero_rows):
        raise RuntimeError("Native support dimension does not match the whole exact-zero blocks")
    for key, expected in (("original_block_count", hessian.bRows()),
                          ("support_block_count", hessian.bRows() - len(checked_blocks))):
        if key in structure and _structure_integer(structure[key], key) != expected:
            raise RuntimeError("Native Hessian structure block count is inconsistent: " + key)
    selected_indices = {block["block_index"] for block in blocks}
    excluded = [block for block in checked_blocks if block["block_index"] not in selected_indices]
    return {
        "native_hessian_structure": structure,
        "support_scalar_dimension": support_dimension,
        "support_block_count": int(hessian.bRows()) - len(checked_blocks),
        "original_hessian_singular": True if zero_rows else None,
        "excluded_exact_zero_nuisance_blocks": excluded,
        "excluded_exact_zero_nuisance_coordinates": [row for block in excluded
                                                     for row in range(block["scalar_base"], block["scalar_base"] + block["dimension"])],
        "selected_exact_zero_blocks": [block for block in checked_blocks if block["block_index"] in selected_indices],
        "unresolved_exact_zero_scalar_rows": sorted(set(zero_rows) - whole_zero_rows),
    }


def analytic_marginal_covariance(calibrator, backend, marginal_covariance,
                                 hessian_structure=None, diagnostic_path=None):
    """Build analytic H; optionally quotient only proven whole exact-zero nuisances.

    Without a structure callback the supplied native solve must use strict full H.
    """
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
    support = {"native_hessian_structure": None, "support_scalar_dimension": rows,
               "support_block_count": int(hessian.bRows()), "original_hessian_singular": None,
               "excluded_exact_zero_nuisance_blocks": [], "excluded_exact_zero_nuisance_coordinates": [],
               "selected_exact_zero_blocks": [], "unresolved_exact_zero_scalar_rows": []}
    if hessian_structure is not None:
        support = _support_structure(hessian, blocks, hessian_structure(hessian))
    quotiented = bool(support["excluded_exact_zero_nuisance_coordinates"])
    metadata = {
        "schema_version": 1,
        "method": ("analytic_hessian_support_quotient_cholmod_selected_columns" if support["original_hessian_singular"]
                   else "full_analytic_hessian_cholmod_selected_columns"),
        "ordering": list(ORDERING),
        "selected_blocks": blocks,
        "selected_block_indices": indices,
        "selected_minimal_dimensions": [block["minimal_dimension"] for block in blocks],
        "full_hessian_dimensions": {"rows": rows, "cols": cols,
                                    "block_rows": int(hessian.bRows()), "block_cols": int(hessian.bCols())},
        **support,
        "exact_zero_policy": {
            "criterion": "native exact-zero coefficient test over entire symmetric rows and columns; no epsilon",
            "whole_nuisance_blocks_only": True,
            "exclude_selected_or_coupled_or_partial_blocks": False,
            "exclude_weak_coordinates": False,
            "all_nonzero_information_and_residual_terms_retained": True,
            "remaining_support_must_be_positive_definite": True,
        },
        "diagonal_damping": 0.0,
        "optimizer_rerun": False,
        "all_problem_error_terms_retained": True,
        "bias_spline_analytic_Q_retained": True,
        "robust_loss_weighting": False,
        "full_nuisance_variables_marginalized": False,
        "support_nuisance_variables_marginalized": False,
        "support_factorization_performed": False,
        "selected_inverse": None,
        "conditional_on_configured_noise_model": True,
        "noise_model_parameter_uncertainty_marginalized": False,
        "interpretation": "Local linearized covariance conditional on supplied fixed IMU noise coefficients; "
                          "engineering prior coefficient uncertainty and systematic errors are not propagated",
    }
    if quotiented:
        metadata["interpretation"] += "; selected covariance is defined from the reduced information support only, " \
                                      "not an inverse or proper marginal covariance of the singular original Hessian"
    # Preserve structure diagnostics even if selected-factorization/validation fails.
    if diagnostic_path is not None:
        with Path(diagnostic_path).open("w", encoding="utf-8") as stream:
            yaml.safe_dump(dict(metadata, diagnostic_stage="before_selected_factorization"), stream, sort_keys=False)
    if support["selected_exact_zero_blocks"]:
        raise RuntimeError("Selected covariance design variables contain exact-zero Hessian blocks")
    if support["unresolved_exact_zero_scalar_rows"]:
        raise RuntimeError("Hessian support retains unresolved partial exact-zero rows; no coordinates were dropped")
    # Native CHOLMOD factors the full information support once and solves selected
    # basis columns. Nonzero nuisance coupling and analytic spline Q stay intact.
    covariance = validate_covariance(marginal_covariance(hessian, indices))
    metadata.update(support_factorization_performed=True,
                    full_nuisance_variables_marginalized=not quotiented,
                    support_nuisance_variables_marginalized=True)
    metadata["selected_inverse"] = ("inverse(H_support)[selected_support_coordinates,selected_support_coordinates]" if quotiented
                                    else "(H^-1)[selected,selected], not inverse(H[selected,selected])")
    if metadata["original_hessian_singular"] is None:
        metadata["original_hessian_singular"] = False
    metadata["validation"] = {"finite": True, "positive_definite": True,
                              "symmetry_rtol": 1e-8, "symmetry_atol": 1e-12,
                              "clipping": False, "pseudoinverse": False}
    return covariance, metadata


def recover_covariance(calibrator):
    output = Path("/data")
    # This is a diagnostic snapshot, never the completed candidate result.
    calibrator.saveCamChainParametersYaml(str(output / "pre_covariance_camchain.yaml"))
    module = importlib.import_module(KALIBR_MODULE)
    extension = importlib.import_module("spr_kalibr_covariance")
    print("Recovering analytic information-support covariance (zero damping)...", flush=True)
    covariance, metadata = analytic_marginal_covariance(
        calibrator, module.aopt, extension.marginal_covariance_on_support,
        hessian_structure=extension.hessian_structure, diagnostic_path=output / "covariance_structure.yaml")
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
