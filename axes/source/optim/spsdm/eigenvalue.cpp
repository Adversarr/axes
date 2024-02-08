#include "axes/optim/spsdm/eigenvalue.hpp"

#include "axes/core/echo.hpp"
#include "axes/utils/status.hpp"

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

void EigenvalueModification::SetOptions(utils::Opt const& options) {
  min_eigval_ = options.Get<real>("min_eigval");
}

utils::Opt EigenvalueModification::GetOptions() const {
  return utils::Opt{{"min_eigval", min_eigval_}};
}

}  // namespace ax::optim
