#include "axes/math/linsys/dense/FullPivHouseholderQR.hpp"

#include "axes/utils/status.hpp"

namespace ax::math {

LinsysSolveResult DenseSolver_FullPivHouseHolderQR::Solve(vecxr const& b, vecxr const&) {
  if (!(impl_.isInjective())) {
    return utils::FailedPreconditionError("The factorization has not been computed.");
  }
  vecxr x = impl_.solve(b);
  return LinsysSolveResultImpl{std::move(x)};
}

Status DenseSolver_FullPivHouseHolderQR::Analyse(problem_t const& problem) {
  impl_.compute(problem.A_);
  if (!(impl_.isInjective())) {
    return utils::FailedPreconditionError("The factorization has not been computed.");
  }
  AX_RETURN_OK();
}
}  // namespace ax::math
