#include "ax/optim/spsdm/eigenvalue.hpp"

#include "ax/core/echo.hpp"
#include "ax/utils/status.hpp"

namespace ax::optim {

StatusOr<math::matxxr> EigenvalueModification::Modify(math::matxxr const& A) {
  Eigen::SelfAdjointEigenSolver<math::matxxr> es;
  es.compute(A, Eigen::ComputeEigenvectors);
  if (es.info() != Eigen::Success) {
    return utils::FailedPreconditionError("Eigenvalue decomposition failed");
  }

  math::vecxr eigvals_mod = es.eigenvalues();

  if ((eigvals_mod.array() > min_eigval_).all()) {
    return A;
  }

  for (idx i = 0; i < eigvals_mod.size(); ++i) {
    eigvals_mod[i] = std::max(eigvals_mod[i], min_eigval_);
  }
  math::matxxr A_mod = es.eigenvectors() * eigvals_mod.asDiagonal()
                       * es.eigenvectors().transpose();

  return A_mod;
}

StatusOr<math::sp_matxxr> EigenvalueModification::Modify(math::sp_matxxr const& A) {
  AX_LOG_FIRST_N(WARNING, 1) << "EigenvalueModification::Modify: "
                             << "This method will convert sparse matrix to a dense matrix, and leading to a large memory usage.";
  math::matxxr A_dense = A;
  auto A_mod = Modify(A_dense);
  if (!A_mod.ok()) {
    return A_mod.status();
  }
  return A_mod->sparseView();
}


Status EigenvalueModification::SetOptions(utils::Opt const& options) {
  AX_SYNC_OPT(options, real, min_eigval);
  AX_RETURN_OK();
}

utils::Opt EigenvalueModification::GetOptions() const {
  return utils::Opt{{"min_eigval", min_eigval_}};
}

}  // namespace ax::optim
