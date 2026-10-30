"""Matrix/ordering/control tests; no Kalibr, sensor data, or result files."""

import ast
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, mock_open, patch

import numpy as np
import yaml

from spr_calibration import kalibr_covariance as adapter


class DesignVariable:
    def __init__(self, index, dimension):
        self.index, self.dimension, self.active, self.scale = index, dimension, True, 1.0

    def blockIndex(self):
        return self.index

    def minimalDimensions(self):
        return self.dimension

    def isActive(self):
        return self.active

    def scaling(self):
        return self.scale


class MatrixLayout:
    # Deliberately interleave a nuisance block and reverse calibration block order.
    dimensions = [1, 3, 1, 3, 1]

    def __init__(self):
        self.matrix = np.eye(9) * 2.0
        self.matrix[4, 5] = self.matrix[5, 4] = 0.5

    def rows(self):
        return self.matrix.shape[0]

    def cols(self):
        return self.matrix.shape[1]

    def bRows(self):
        return len(self.dimensions)

    def bCols(self):
        return len(self.dimensions)

    def rowsOfBlock(self, index):
        return self.dimensions[index]

    def colsOfBlock(self, index):
        return self.dimensions[index]

    def rowBaseOfBlock(self, index):
        return sum(self.dimensions[:index])


class KalibrCovarianceTests(unittest.TestCase):
    def setUp(self):
        self.rotation, self.translation = DesignVariable(3, 3), DesignVariable(1, 3)
        self.time0, self.time1 = DesignVariable(4, 1), DesignVariable(0, 1)
        transform = SimpleNamespace(getDesignVariable=lambda index: (self.rotation, self.translation)[index])
        self.chain = SimpleNamespace(camList=[
            SimpleNamespace(T_c_b_Dv=transform, cameraTimeToImuTimeDv=self.time0),
            SimpleNamespace(cameraTimeToImuTimeDv=self.time1)])
        self.calibrator = SimpleNamespace(problem=object(), CameraChain=self.chain)
        self.hessian = MatrixLayout()

    def scalar_indices(self, block_indices):
        return [index for block in block_indices
                for index in range(self.hessian.rowBaseOfBlock(block),
                                   self.hessian.rowBaseOfBlock(block) + self.hessian.rowsOfBlock(block))]

    def backend(self, events):
        owner = self

        class Optimizer:
            def __init__(self, options):
                events.append(("options", options))

            def setProblem(self, problem):
                events.append(("problem", problem))

            def initialize(self):
                events.append(("initialize",))

            def computeHessian(self, hessian, damping):
                events.append(("hessian", hessian, damping))

            def optimize(self):
                raise AssertionError("Covariance must never optimize again")

        return SimpleNamespace(Optimizer2Options=SimpleNamespace,
                               BlockCholeskyLinearSystemSolver=object,
                               Optimizer2=Optimizer, SparseBlockMatrixXd=lambda: owner.hessian)

    def test_full_marginal_keeps_nuisance_coupling_and_requested_unsorted_order(self):
        events = []
        requested = []

        def selected_inverse(hessian, indices):
            requested.append(list(indices))
            self.assertIs(hessian, self.hessian)
            selected = self.scalar_indices(indices)
            # Pure matrix identity: solve FULL H against selected basis columns.
            solutions = np.linalg.solve(hessian.matrix, np.eye(hessian.rows())[:, selected])
            return solutions[selected, :].tolist()

        covariance, metadata = adapter.analytic_marginal_covariance(
            self.calibrator, self.backend(events), selected_inverse)
        self.assertEqual(requested, [[3, 1, 4, 0]])
        self.assertAlmostEqual(covariance[0, 0], 1.0 / (2.0 - 0.5**2 / 2.0))
        selected = self.scalar_indices(requested[0])
        conditioned = np.linalg.solve(self.hessian.matrix[np.ix_(selected, selected)], np.eye(8))
        self.assertNotAlmostEqual(covariance[0, 0], conditioned[0, 0])
        self.assertEqual(metadata["selected_minimal_dimensions"], [3, 3, 1, 1])
        self.assertEqual(metadata["full_hessian_dimensions"], {"rows": 9, "cols": 9, "block_rows": 5, "block_cols": 5})
        self.assertEqual(metadata["ordering"], adapter.ORDERING)
        self.assertTrue(metadata["bias_spline_analytic_Q_retained"])
        self.assertTrue(metadata["full_nuisance_variables_marginalized"])
        self.assertTrue(metadata["conditional_on_configured_noise_model"])
        self.assertFalse(metadata["noise_model_parameter_uncertainty_marginalized"])
        self.assertEqual(events[1:], [("problem", self.calibrator.problem), ("initialize",),
                                      ("hessian", self.hessian, 0.0)])
        self.assertFalse(events[0][1].doSchurComplement)

    def test_indices_are_read_after_optimizer_initialization(self):
        events = []
        backend = self.backend(events)
        base_optimizer = backend.Optimizer2
        self.rotation.index = -1

        class Optimizer(base_optimizer):
            def initialize(inner):
                super().initialize()
                self.rotation.index = 3

        backend.Optimizer2 = Optimizer
        covariance, _ = adapter.analytic_marginal_covariance(
            self.calibrator, backend, lambda hessian, indices: np.eye(8))
        self.assertEqual(covariance.shape, (8, 8))

    def test_inactive_wrong_dimension_duplicate_and_nonunit_blocks_are_rejected(self):
        for attribute, value, message in (("active", False, "inactive"), ("dimension", 2, "dimension mismatch"),
                                           ("index", -1, "outside"), ("index", True, "integer"),
                                           ("scale", 2.0, "unit design-variable scaling")):
            with self.subTest(attribute=attribute):
                self.setUp()
                setattr(self.rotation, attribute, value)
                with self.assertRaisesRegex(RuntimeError, message):
                    adapter.calibration_blocks(self.chain, self.hessian)
        self.setUp()
        self.time0.index = self.time1.index
        with self.assertRaisesRegex(RuntimeError, "duplicate"):
            adapter.calibration_blocks(self.chain, self.hessian)

    def test_mismatched_matrix_block_dimension_is_rejected(self):
        self.hessian.dimensions = [1, 2, 1, 3, 1]
        with self.assertRaisesRegex(RuntimeError, "dimension mismatch"):
            adapter.calibration_blocks(self.chain, self.hessian)

    def test_only_stereo_camera_selection_is_supported(self):
        self.chain.camList = self.chain.camList[:1]
        with self.assertRaisesRegex(RuntimeError, "exactly two"):
            adapter.calibration_blocks(self.chain, self.hessian)

    def test_invalid_full_hessian_never_reaches_native_factorization(self):
        self.hessian.matrix = np.ones((9, 10))
        native = Mock()
        with self.assertRaisesRegex(RuntimeError, "square"):
            adapter.analytic_marginal_covariance(self.calibrator, self.backend([]), native)
        native.assert_not_called()

    def test_invalid_covariance_is_rejected_without_repair(self):
        matrices = [(np.eye(7), "finite 8x8"), (np.full((8, 8), np.nan), "finite 8x8"),
                    (np.zeros((8, 8)), "positive definite"), (-np.eye(8), "positive definite"),
                    (np.eye(8, dtype=complex), "real")]
        asymmetric = np.eye(8)
        asymmetric[0, 1] = 0.1
        matrices.append((asymmetric, "symmetric"))
        for matrix, message in matrices:
            with self.subTest(message=message):
                original = matrix.copy()
                with self.assertRaisesRegex(RuntimeError, message):
                    adapter.validate_covariance(matrix)
                np.testing.assert_equal(matrix, original)

    def test_positive_covariance_is_returned_unmodified(self):
        covariance = np.diag(np.arange(1.0, 9.0))
        original = covariance.copy()
        result = adapter.validate_covariance(covariance)
        np.testing.assert_array_equal(result, original)

    def test_diagnostic_snapshot_precedes_native_import_failure(self):
        events = []
        self.calibrator.saveCamChainParametersYaml = lambda path: events.append(("snapshot", path))

        def import_failure(name):
            events.append(("import", name))
            raise RuntimeError("backend unavailable")

        with patch.object(adapter.importlib, "import_module", side_effect=import_failure):
            with self.assertRaisesRegex(RuntimeError, "backend unavailable"):
                adapter.recover_covariance(self.calibrator)
        self.assertEqual(events[0], ("snapshot", "/data/pre_covariance_camchain.yaml"))
        self.assertFalse(hasattr(self.calibrator, "std_trafo_ic"))

    def test_upstream_standard_deviation_split_and_metadata_output(self):
        covariance = np.diag(np.arange(1.0, 9.0))
        self.calibrator.saveCamChainParametersYaml = Mock()
        module = SimpleNamespace(aopt=object())
        extension = SimpleNamespace(marginal_covariance=object())
        opened = mock_open()
        with patch.object(adapter.importlib, "import_module", side_effect=[module, extension]), \
                patch.object(adapter, "analytic_marginal_covariance", return_value=(covariance, {"control_metadata": True})), \
                patch.object(adapter.np, "save") as save, patch.object(Path, "open", opened), redirect_stdout(StringIO()):
            adapter.recover_covariance(self.calibrator)
        save.assert_called_once()
        self.assertEqual(save.call_args.args[0], "/data/joint_covariance.npy")
        np.testing.assert_allclose(self.calibrator.std_trafo_ic**2, np.arange(1.0, 7.0))
        np.testing.assert_allclose(self.calibrator.std_times**2, [7.0, 8.0])
        written = "".join(call.args[0] for call in opened().write.call_args_list)
        self.assertEqual(yaml.safe_load(written), {"control_metadata": True})

    def test_adapter_uses_python38_grammar_and_has_no_relative_imports(self):
        source = Path(adapter.__file__).read_text(encoding="utf-8")
        tree = ast.parse(source, feature_version=(3, 8))
        self.assertFalse(any(isinstance(node, ast.ImportFrom) and node.level for node in ast.walk(tree)))


if __name__ == "__main__":
    unittest.main()
