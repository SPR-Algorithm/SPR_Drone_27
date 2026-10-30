"""Matrix-only marginalization regressions; no sensor data are generated."""

import unittest

import aslam_backend as backend
import numpy as np
from spr_kalibr_covariance import hessian_structure, marginal_covariance, marginal_covariance_on_support


def block_matrix(values, layout):
    layout = np.asarray(layout, dtype=np.int32)
    matrix = backend.SparseBlockMatrixXd(layout, layout, True)
    starts = np.r_[0, layout[:-1]]
    for row in range(len(layout)):
        for col in range(row, len(layout)):
            matrix.setBlock(row, col, np.asarray(values[starts[row]:layout[row], starts[col]:layout[col]], dtype=float))
    return matrix


class MarginalCovarianceTests(unittest.TestCase):
    def setUp(self):
        self.full = np.array([[4., .8, .5, .2], [.8, 3., .3, .1],
                              [.5, .3, 2., .4], [.2, .1, .4, 1.5]])
        self.hessian = block_matrix(self.full, [1, 3, 4])

    def test_permuted_selected_blocks_equal_full_inverse(self):
        actual = np.asarray(marginal_covariance(self.hessian, [2, 0]))
        expected = np.linalg.inv(self.full)[np.ix_([3, 0], [3, 0])]
        np.testing.assert_allclose(actual, expected, atol=1e-12)

    def test_nuisance_correlations_are_not_conditioned_away(self):
        actual = np.asarray(marginal_covariance(self.hessian, [1]))
        expected = np.linalg.inv(self.full)[1:3, 1:3]
        np.testing.assert_allclose(actual, expected, atol=1e-12)
        self.assertFalse(np.allclose(actual, np.linalg.inv(self.full[1:3, 1:3])))

    def test_singular_full_system_cannot_return_an_invented_covariance(self):
        singular = block_matrix(np.diag([1., 0.]), [1, 2])
        with self.assertRaises(RuntimeError):
            marginal_covariance(singular, [0])

    def test_negative_nuisance_direction_is_rejected_even_when_selected_inverse_is_positive(self):
        indefinite = block_matrix(np.diag([1., -1.]), [1, 2])
        with self.assertRaises(RuntimeError):
            marginal_covariance(indefinite, [0])

    def test_invalid_or_duplicate_block_requests_are_rejected(self):
        for indices in ([], [3], [-1], [0, 0], [True]):
            with self.subTest(indices=indices), self.assertRaises(Exception):
                marginal_covariance(self.hessian, indices)

    def test_actual_hessian_dimension_does_not_overflow_inverse_cache_keys(self):
        # Upstream's int key r*n+c collides for (58498,58498) and (0,14362)
        # at this dimension. A sparse diagonal matrix needs no dense N x N data.
        size = 73420
        layout = np.arange(1, size + 1, dtype=np.int32)
        hessian = backend.SparseBlockMatrixXd(layout, layout, True)
        unit = np.ones((1, 1))
        for index in range(size):
            hessian.setBlock(index, index, unit)
        hessian.setBlock(58498, 58498, np.array([[2.]]))
        hessian.setBlock(14362, 14362, np.array([[3.]]))
        actual = np.asarray(marginal_covariance(hessian, [58498, 0, 14362]))
        np.testing.assert_allclose(actual, np.diag([.5, 1., 1. / 3.]), atol=1e-12)

    def test_only_exactly_uncoupled_unused_nuisance_blocks_can_be_quotiented(self):
        full = np.zeros((5, 5))
        full[:4, :4] = self.full
        hessian = block_matrix(full, [1, 3, 4, 5])
        structure = hessian_structure(hessian)
        self.assertEqual(structure['exact_zero_blocks'], [{'block_index': 3, 'scalar_base': 4, 'dimension': 1}])
        self.assertEqual(structure['exact_zero_scalar_rows'], [4])
        self.assertEqual(structure['support_scalar_dimension'], 4)
        actual = np.asarray(marginal_covariance_on_support(hessian, [2, 0]))
        np.testing.assert_allclose(actual, np.linalg.inv(self.full)[np.ix_([3, 0], [3, 0])], atol=1e-12)

    def test_requested_zero_information_cannot_be_quotiented_away(self):
        hessian = block_matrix(np.diag([1., 0.]), [1, 2])
        with self.assertRaisesRegex(RuntimeError, 'calibration block'):
            marginal_covariance_on_support(hessian, [1])

    def test_coupled_null_directions_are_not_pruned(self):
        hessian = block_matrix(np.ones((2, 2)), [1, 2])
        self.assertEqual(hessian_structure(hessian)['exact_zero_blocks'], [])
        with self.assertRaises(RuntimeError):
            marginal_covariance_on_support(hessian, [0])

    def test_weak_nonzero_information_is_retained(self):
        hessian = block_matrix(np.diag([1., 1e-15]), [1, 2])
        self.assertEqual(hessian_structure(hessian)['exact_zero_blocks'], [])
        np.testing.assert_allclose(marginal_covariance_on_support(hessian, [1]), [[1e15]], rtol=1e-12)


if __name__ == "__main__":
    unittest.main()
