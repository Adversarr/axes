#include "ax/math/linsys/dense/FullPivHouseholderQR.hpp"

namespace ax::math {

math::vecxr DenseSolver_FullPivHouseHolderQR::Solve(vecxr const& b) {
  AX_THROW_IF_FALSE(impl_.isInjective(), "The factorization has not been computed.");
  vecxr x = impl_.solve(b);
  return x;
}

void DenseSolver_FullPivHouseHolderQR::Compute() {
  impl_.compute(cached_problem_->A_);
  AX_THROW_IF_FALSE(impl_.isInjective(), "The factorization has not been computed.");
}
}  // namespace ax::math
