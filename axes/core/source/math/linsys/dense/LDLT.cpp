#include "ax/math/linsys/dense/LDLT.hpp"

#include "ax/utils/status.hpp"

namespace ax::math {

LinsysSolveResult DenseSolver_LDLT::Solve(vecxr const& b, vecxr const&) {
  if (!(impl_.info() == Eigen::Success)) {
    return utils::FailedPreconditionError("The factorization has not been computed.");
  }
  vecxr x = impl_.solve(b);
  return LinsysSolveResultImpl{std::move(x)};
}

Status DenseSolver_LDLT::Analyse(problem_t const& problem) {
  impl_.compute(problem.A_);
  if (!(impl_.info() == Eigen::Success)) {
    return utils::FailedPreconditionError("The factorization has not been computed.");
  } else if (!impl_.isPositive()) {
    return utils::FailedPreconditionError("The matrix is not positive definite.");
  }
  AX_RETURN_OK();
}
}  // namespace ax::math
