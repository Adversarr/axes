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

Status EigenvalueModification::SetOptions(utils::Opt const& options) {
  AX_SYNC_OPT(options, real, min_eigval);
  AX_RETURN_OK();
}

utils::Opt EigenvalueModification::GetOptions() const {
  return utils::Opt{{"min_eigval", min_eigval_}};
}

}  // namespace ax::optim
