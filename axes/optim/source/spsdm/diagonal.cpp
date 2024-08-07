#include "ax/optim/spsdm/diagonal.hpp"

#include "ax/core/echo.hpp"
#include "ax/utils/status.hpp"

namespace ax::optim {

void DiagonalModification::SetOptions(utils::Options const& options) {
  AX_SYNC_OPT_IF(options, real, additional_offset) {
    AX_THROW_IF_LT(additional_offset_, 0, "Eigen modification encountered negative offset");
  }
  SpsdModificationBase::SetOptions(options);
}

utils::Options DiagonalModification::GetOptions() const {
  utils::Options opt = SpsdModificationBase::GetOptions();
  opt.insert_or_assign("additional_offset", additional_offset_);
  return opt;
}

math::matxxr DiagonalModification::Modify(math::matxxr const& A) {
  math::vecxr row_sum(A.rows());
  math::vecxr col_sum(A.cols());
  for (idx i = 0; i < A.rows(); ++i) {
    for (idx j = 0; j < A.cols(); ++j) {
      real abs_aij = std::abs(A(i, j));
      row_sum[i] += abs_aij;
      col_sum[j] += abs_aij;
    }
  }

  math::matxxr A_mod = A;
  for (idx i = 0; i < A.rows(); ++i) {
    real diag = abs(A(i, i));
    diag = std::max(diag, row_sum[i] - diag + additional_offset_);
    diag = std::max(diag, col_sum[i] - diag + additional_offset_);
    A_mod(i, i) = diag;
  }

  return A_mod;
}

math::spmatr DiagonalModification::Modify(math::spmatr const& A) {
  math::vecxr row_sum(A.rows());
  math::vecxr col_sum(A.cols());
  for (idx i = 0; i < A.outerSize(); ++i) {
    for (math::spmatr::InnerIterator it(A, i); it; ++it) {
      real abs_aij = std::abs(it.value());
      row_sum[it.row()] += abs_aij;
      col_sum[it.col()] += abs_aij;
    }
  }

  math::spmatr A_mod = A;
  for (idx i = 0; i < A.rows(); ++i) {
    real diag = abs(A.coeff(i, i));
    diag = std::max(diag, row_sum[i] - diag + additional_offset_);
    diag = std::max(diag, col_sum[i] - diag + additional_offset_);
    A_mod.coeffRef(i, i) = diag;
  }

  return A_mod;
}

}
