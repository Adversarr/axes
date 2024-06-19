#include "ax/math/linsys/preconditioner/IncompleteCholesky.hpp"

#include "ax/utils/status.hpp"

namespace ax::math {

void Preconditioner_IncompleteCholesky::Analyse(LinsysProblem_Sparse const &problem) {
  impl_.compute(problem.A_);
  // if (!(impl_.info() == Eigen::Success)) {
  //   return utils::FailedPreconditionError("The factorization has not been computed.");
  // }
  // AX_RETURN_OK();
  AX_THROW_IF_FALSE(impl_.info() == Eigen::Success, "The factorization has not been computed.");
}

vecxr Preconditioner_IncompleteCholesky::Solve(vecxr const &b, vecxr const &) {
  return impl_.solve(b);
}

}  // namespace ax::math