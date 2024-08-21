#include "ax/optim/spsdm/diagonal.hpp"

#include "ax/core/logging.hpp"

namespace ax::optim {

void DiagonalModification::SetOptions(utils::Options const& options) {
  AX_SYNC_OPT_IF(options, Real, additional_offset) {
    AX_THROW_IF_LT(additional_offset_, 0, "Eigen modification encountered negative offset");
  }
  SpsdModificationBase::SetOptions(options);
}

utils::Options DiagonalModification::GetOptions() const {
  utils::Options opt = SpsdModificationBase::GetOptions();
  opt.insert_or_assign("additional_offset", additional_offset_);
  return opt;
}

math::RealMatrixX DiagonalModification::Modify(math::RealMatrixX const& A) {
  math::RealVectorX row_sum(A.rows());
  math::RealVectorX col_sum(A.cols());
  for (Index i = 0; i < A.rows(); ++i) {
    for (Index j = 0; j < A.cols(); ++j) {
      Real abs_aij = std::abs(A(i, j));
      row_sum[i] += abs_aij;
      col_sum[j] += abs_aij;
    }
  }

  math::RealMatrixX A_mod = A;
  for (Index i = 0; i < A.rows(); ++i) {
    Real diag = abs(A(i, i));
    diag = std::max(diag, row_sum[i] - diag + additional_offset_);
    diag = std::max(diag, col_sum[i] - diag + additional_offset_);
    A_mod(i, i) = diag;
  }

  return A_mod;
}

math::RealSparseMatrix DiagonalModification::Modify(math::RealSparseMatrix const& A) {
  math::RealVectorX row_sum(A.rows());
  math::RealVectorX col_sum(A.cols());
  for (Index i = 0; i < A.outerSize(); ++i) {
    for (math::RealSparseMatrix::InnerIterator it(A, i); it; ++it) {
      Real abs_aij = std::abs(it.value());
      row_sum[it.row()] += abs_aij;
      col_sum[it.col()] += abs_aij;
    }
  }

  math::RealSparseMatrix A_mod = A;
  for (Index i = 0; i < A.rows(); ++i) {
    Real diag = abs(A.coeff(i, i));
    diag = std::max(diag, row_sum[i] - diag + additional_offset_);
    diag = std::max(diag, col_sum[i] - diag + additional_offset_);
    A_mod.coeffRef(i, i) = diag;
  }

  return A_mod;
}

}
