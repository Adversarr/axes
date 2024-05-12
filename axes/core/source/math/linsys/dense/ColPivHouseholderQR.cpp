#include "ax/math/linsys/dense/ColPivHouseholderQR.hpp"

#include "ax/utils/status.hpp"

namespace ax::math {

LinsysSolveResult DenseSolver_ColPivHouseholderQR::Solve(vecxr const& b, vecxr const&) {
  AX_THROW_IF_FALSE(impl_.isInjective(), "The factorization has not been computed.");
  vecxr x = impl_.solve(b);
  return LinsysSolveResult{std::move(x)};
}

void DenseSolver_ColPivHouseholderQR::Analyse(problem_t const& problem) {
  impl_.compute(problem.A_);
  AX_THROW_IF_FALSE(impl_.isInjective(), "The factorization has not been computed.");
}

}  // namespace ax::math
