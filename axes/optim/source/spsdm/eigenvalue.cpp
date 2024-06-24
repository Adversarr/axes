#include "ax/optim/spsdm/eigenvalue.hpp"

#include "ax/core/echo.hpp"
#include "ax/core/excepts.hpp"
#include "ax/optim/spsdm.hpp"
#include "ax/utils/status.hpp"

namespace ax::optim {

math::matxxr EigenvalueModification::Modify(math::matxxr const& A) {
  Eigen::SelfAdjointEigenSolver<math::matxxr> es;
  es.compute(A, Eigen::ComputeEigenvectors);
  AX_THROW_IF_FALSE(es.info() == Eigen::Success, "Eigenvalue decomposition failed.");
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

math::spmatr EigenvalueModification::Modify(math::spmatr const& A) {
  AX_LOG_FIRST_N(WARNING, 1) << "EigenvalueModification::Modify: "
                             << "This method will convert sparse matrix to a dense matrix, and leading to a large memory usage.";
  math::matxxr A_dense = A;
  auto A_mod = Modify(A_dense);
  return A_mod.sparseView();
}


void EigenvalueModification::SetOptions(utils::Options const& options) {
  AX_SYNC_OPT(options, real, min_eigval);
  SpsdModificationBase::SetOptions(options);
}

utils::Options EigenvalueModification::GetOptions() const {
  return utils::Options{{"min_eigval", min_eigval_}};
}

}  // namespace ax::optim
