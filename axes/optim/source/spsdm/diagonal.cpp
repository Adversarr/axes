#include "ax/optim/spsdm/diagonal.hpp"

#include "ax/core/echo.hpp"
#include "ax/utils/status.hpp"

namespace ax::optim {

Status DiagonalModification::SetOptions(utils::Opt const& options) {
  AX_SYNC_OPT_IF(options, real, additional_offset) {
    if (additional_offset_ < 0) {
      return utils::InvalidArgumentError("additional_offset must be non-negative.");
    }
  }
  return SpsdModificationBase::SetOptions(options);
}

utils::Opt DiagonalModification::GetOptions() const {
  utils::Opt opt = SpsdModificationBase::GetOptions();
  opt.insert_or_assign("additional_offset", additional_offset_);
  return opt;
}

StatusOr<math::matxxr> DiagonalModification::Modify(math::matxxr const& A) {
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

StatusOr<math::sp_matxxr> DiagonalModification::Modify(math::sp_matxxr const& A) {
  math::vecxr row_sum(A.rows());
  math::vecxr col_sum(A.cols());
  for (idx i = 0; i < A.outerSize(); ++i) {
    for (math::sp_matxxr::InnerIterator it(A, i); it; ++it) {
      real abs_aij = std::abs(it.value());
      row_sum[it.row()] += abs_aij;
      col_sum[it.col()] += abs_aij;
    }
  }

  math::sp_matxxr A_mod = A;
  for (idx i = 0; i < A.rows(); ++i) {
    real diag = abs(A.coeff(i, i));
    diag = std::max(diag, row_sum[i] - diag + additional_offset_);
    diag = std::max(diag, col_sum[i] - diag + additional_offset_);
    A_mod.coeffRef(i, i) = diag;
  }

  return A_mod;
}

}