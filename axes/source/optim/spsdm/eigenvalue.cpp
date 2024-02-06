#include "axes/optim/spsdm/eigenvalue.hpp"

#include "axes/core/echo.hpp"
#include "axes/utils/status.hpp"

namespace ax::optim {

StatusOr<math::matxxr> EigenvalueModification::Modify(math::matxxr const& A,
                                                      utils::Opt const& opt) {
  real min_eigval = opt.Get<real>("min_eigval", min_eigval_);
  Eigen::SelfAdjointEigenSolver<math::matxxr> es;
  es.compute(A, Eigen::ComputeEigenvectors);
  if (es.info() != Eigen::Success) {
    return utils::FailedPreconditionError("Eigenvalue decomposition failed");
  }

  math::vecxr eigvals_mod = es.eigenvalues();

  if ((eigvals_mod.array() > min_eigval).all()) {
    return A;
  }

  for (idx i = 0; i < eigvals_mod.size(); ++i) {
    eigvals_mod[i] = std::max(eigvals_mod[i], min_eigval);
  }
  math::matxxr A_mod = es.eigenvectors() * eigvals_mod.asDiagonal()
                       * es.eigenvectors().transpose();

  return A_mod;
}

}  // namespace ax::optim
