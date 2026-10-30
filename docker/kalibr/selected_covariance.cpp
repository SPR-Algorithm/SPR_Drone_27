#include <boost/python.hpp>
#include <sparse_block_matrix/linear_solver_cholmod.h>

#include <stdexcept>
#include <utility>
#include <vector>

namespace {
using BlockMatrix = sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd>;
namespace python = boost::python;

class FullHessianSolver : public sparse_block_matrix::LinearSolverCholmod<Eigen::MatrixXd> {
 public:
  Eigen::MatrixXd SolveColumns(const BlockMatrix& hessian, Eigen::MatrixXd& basis) {
    // Simplicial LDL' can successfully factor negative pivots. Require LL'
    // so an indefinite nuisance direction cannot escape a selected-block check.
    _cholmodCommon.final_asis = 0;
    _cholmodCommon.final_ll = 1;
    fillCholmodExt(hessian, false);
    computeSymbolicDecomposition(hessian);
    if (_cholmodFactor == nullptr || _cholmodCommon.status != CHOLMOD_OK) {
      throw std::runtime_error("Full Hessian symbolic factorization failed");
    }
    if (!cholmod_factorize(_cholmodSparse, _cholmodFactor, &_cholmodCommon) ||
        _cholmodCommon.status != CHOLMOD_OK || !_cholmodFactor->is_ll ||
        _cholmodFactor->minor != _cholmodSparse->nrow) {
      throw std::runtime_error("Full Hessian is not positive definite; covariance is unavailable");
    }
    cholmod_dense rhs{};
    rhs.nrow = rhs.d = static_cast<std::size_t>(basis.rows());
    rhs.ncol = static_cast<std::size_t>(basis.cols());
    rhs.nzmax = static_cast<std::size_t>(basis.size());
    rhs.x = basis.data();
    rhs.xtype = CHOLMOD_REAL;
    rhs.dtype = CHOLMOD_DOUBLE;
    cholmod_dense* solution = cholmod_solve(CHOLMOD_A, _cholmodFactor, &rhs, &_cholmodCommon);
    if (solution == nullptr || _cholmodCommon.status != CHOLMOD_OK) {
      if (solution != nullptr) {
        cholmod_free_dense(&solution, &_cholmodCommon);
      }
      throw std::runtime_error("Full Hessian selected-column solve failed");
    }
    using StridedMatrix = Eigen::Map<const Eigen::MatrixXd, 0, Eigen::OuterStride<>>;
    Eigen::MatrixXd columns = StridedMatrix(static_cast<const double*>(solution->x),
        basis.rows(), basis.cols(), Eigen::OuterStride<>(static_cast<Eigen::Index>(solution->d)));
    cholmod_free_dense(&solution, &_cholmodCommon);
    if (!columns.allFinite()) {
      throw std::runtime_error("Selected inverse columns contain nonfinite values");
    }
    return columns;
  }
};

python::list MarginalCovariance(const BlockMatrix& hessian, python::list requested) {
  if (hessian.rows() == 0 || hessian.rowBlockIndices() != hessian.colBlockIndices()) {
    throw std::invalid_argument("The full Hessian must have a nonempty square block layout");
  }
  std::vector<int> blocks;
  std::vector<int> offsets{0};
  for (int index = 0; index < python::len(requested); ++index) {
    const python::object requested_block = requested[index];
    if (PyBool_Check(requested_block.ptr())) {
      throw std::invalid_argument("Block indices must be integers, not booleans");
    }
    const int block = python::extract<int>(requested_block);
    if (block < 0 || block >= hessian.bRows()) {
      throw std::invalid_argument("Calibration block index is outside the full Hessian");
    }
    for (int previous : blocks) {
      if (previous == block) {
        throw std::invalid_argument("Calibration block indices must be distinct");
      }
    }
    blocks.push_back(block);
    offsets.push_back(offsets.back() + hessian.rowsOfBlock(block));
  }
  if (blocks.empty()) {
    throw std::invalid_argument("At least one calibration block is required");
  }

  for (const auto& column : hessian.blockCols()) {
    for (const auto& entry : column) {
      if (!entry.second->allFinite()) {
        throw std::invalid_argument("Full Hessian contains nonfinite values");
      }
    }
  }
  // Solve H X = E for the selected basis columns after one full factorization.
  // This avoids both a dense full inverse and upstream's 32-bit r*n+c cache
  // keys, which overflow for the actual 73,420-dimensional normal matrix.
  Eigen::MatrixXd basis = Eigen::MatrixXd::Zero(hessian.rows(), offsets.back());
  for (std::size_t index = 0; index < blocks.size(); ++index) {
    const int dimension = hessian.rowsOfBlock(blocks[index]);
    basis.block(hessian.rowBaseOfBlock(blocks[index]), offsets[index], dimension, dimension).setIdentity();
  }
  FullHessianSolver solver;
  const Eigen::MatrixXd columns = solver.SolveColumns(hessian, basis);
  Eigen::MatrixXd covariance(offsets.back(), offsets.back());
  for (std::size_t row = 0; row < blocks.size(); ++row) {
    covariance.middleRows(offsets[row], hessian.rowsOfBlock(blocks[row])) =
        columns.middleRows(hessian.rowBaseOfBlock(blocks[row]), hessian.rowsOfBlock(blocks[row]));
  }
  python::list result;
  for (Eigen::Index row = 0; row < covariance.rows(); ++row) {
    python::list values;
    for (Eigen::Index col = 0; col < covariance.cols(); ++col) {
      values.append(covariance(row, col));
    }
    result.append(values);
  }
  return result;
}
}  // namespace

BOOST_PYTHON_MODULE(spr_kalibr_covariance) {
  boost::python::def("marginal_covariance", MarginalCovariance);
}
