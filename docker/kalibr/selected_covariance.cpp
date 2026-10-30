#include <boost/python.hpp>
#include <sparse_block_matrix/linear_solver_cholmod.h>

#include <stdexcept>
#include <utility>
#include <vector>

namespace {
using BlockMatrix = sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd>;
namespace python = boost::python;

struct InformationSupport {
  std::vector<bool> blocks;
  std::vector<bool> rows;
};

InformationSupport InspectSupport(const BlockMatrix& hessian) {
  if (hessian.rows() == 0 || hessian.rowBlockIndices() != hessian.colBlockIndices()) {
    throw std::invalid_argument("The full Hessian must have a nonempty square block layout");
  }
  InformationSupport support{std::vector<bool>(hessian.bRows(), false),
                             std::vector<bool>(hessian.rows(), false)};
  for (int col = 0; col < hessian.bCols(); ++col) {
    for (const auto& entry : hessian.blockCols()[col]) {
      if (entry.second == nullptr || !entry.second->allFinite()) {
        throw std::invalid_argument("Full Hessian contains missing or nonfinite coefficients");
      }
      const int row = entry.first;
      for (Eigen::Index i = 0; i < entry.second->rows(); ++i) {
        for (Eigen::Index j = 0; j < entry.second->cols(); ++j) {
          if ((*entry.second)(i, j) != 0.0) {
            support.blocks[row] = support.blocks[col] = true;
            support.rows[hessian.rowBaseOfBlock(row) + i] = true;
            support.rows[hessian.colBaseOfBlock(col) + j] = true;
          }
        }
      }
    }
  }
  return support;
}

python::dict HessianStructure(const BlockMatrix& hessian) {
  const auto support = InspectSupport(hessian);
  python::list zero_blocks;
  int supported_rows = 0;
  int supported_blocks = 0;
  for (int block = 0; block < hessian.bRows(); ++block) {
    if (support.blocks[block]) {
      supported_rows += hessian.rowsOfBlock(block);
      ++supported_blocks;
    } else {
      python::dict item;
      item["block_index"] = block;
      item["scalar_base"] = hessian.rowBaseOfBlock(block);
      item["dimension"] = hessian.rowsOfBlock(block);
      zero_blocks.append(item);
    }
  }
  python::list zero_rows;
  for (int row = 0; row < hessian.rows(); ++row) {
    if (!support.rows[row]) {
      zero_rows.append(row);
    }
  }
  python::list dimensions;
  dimensions.append(hessian.rows());
  dimensions.append(hessian.cols());
  python::dict result;
  result["full_dimensions"] = dimensions;
  result["exact_zero_blocks"] = zero_blocks;
  result["exact_zero_scalar_rows"] = zero_rows;
  result["support_scalar_dimension"] = supported_rows;
  result["support_block_count"] = supported_blocks;
  result["original_block_count"] = hessian.bRows();
  result["original_singular_due_to_exact_zero_blocks"] = supported_blocks != hessian.bRows();
  return result;
}

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

python::list MarginalCovarianceImpl(const BlockMatrix& hessian, python::list requested,
                                   bool quotient_exact_zero_nuisance) {
  const auto support = InspectSupport(hessian);
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
  if (quotient_exact_zero_nuisance) {
    std::vector<int> remap(hessian.bRows(), -1);
    std::vector<int> layout;
    int scalar_size = 0;
    for (int block = 0; block < hessian.bRows(); ++block) {
      if (support.blocks[block]) {
        remap[block] = static_cast<int>(layout.size());
        scalar_size += hessian.rowsOfBlock(block);
        layout.push_back(scalar_size);
      }
    }
    for (int block : blocks) {
      if (remap[block] < 0) {
        throw std::runtime_error("Requested calibration block has exactly zero information");
      }
    }
    if (static_cast<int>(layout.size()) != hessian.bRows()) {
      // Only whole rows AND columns whose every coefficient is exactly zero
      // are excluded. All nonzero information and all residual terms remain.
      // The original matrix is singular; this is an inverse on its support.
      BlockMatrix retained(layout, layout);
      for (int col = 0; col < hessian.bCols(); ++col) {
        for (const auto& entry : hessian.blockCols()[col]) {
          if (remap[entry.first] >= 0 && remap[col] >= 0) {
            *retained.block(remap[entry.first], remap[col], true) = *entry.second;
          }
        }
      }
      python::list remapped_request;
      for (int block : blocks) {
        remapped_request.append(remap[block]);
      }
      return MarginalCovarianceImpl(retained, remapped_request, false);
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

python::list MarginalCovariance(const BlockMatrix& hessian, python::list requested) {
  return MarginalCovarianceImpl(hessian, requested, false);
}

python::list MarginalCovarianceOnSupport(const BlockMatrix& hessian, python::list requested) {
  return MarginalCovarianceImpl(hessian, requested, true);
}
}  // namespace

BOOST_PYTHON_MODULE(spr_kalibr_covariance) {
  boost::python::def("marginal_covariance", MarginalCovariance);
  boost::python::def("marginal_covariance_on_support", MarginalCovarianceOnSupport);
  boost::python::def("hessian_structure", HessianStructure);
}
