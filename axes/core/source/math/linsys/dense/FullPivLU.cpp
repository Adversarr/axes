#include "ax/math/linsys/dense/FullPivLU.hpp"
#include "ax/utils/status.hpp"

namespace ax::math {
LinsysSolveResult DenseSolver_FullPivLU::Solve(vecxr const& b, vecxr const&) {
  // if (!(impl_.isInjective())) {
  //   return utils::FailedPreconditionError("The factorization has not been computed.");
  // }
  AX_THROW_IF_FALSE(impl_.isInjective(), "The factorization has not been computed.");
  vecxr x = impl_.solve(b);
  return LinsysSolveResult{std::move(x)};
}

void DenseSolver_FullPivLU::Analyse(problem_t const& problem) {
  impl_.compute(problem.A_);
  // if (!impl_.isInjective()) {
  //   return utils::FailedPreconditionError("The factorization has not been computed.");
  // }
  // AX_RETURN_OK();
  AX_THROW_IF_FALSE(impl_.isInjective(), "The factorization has not been computed.");
}

}  // namespace ax::math
