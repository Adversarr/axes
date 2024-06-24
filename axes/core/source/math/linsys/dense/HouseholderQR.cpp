#include "ax/math/linsys/dense/HouseholderQR.hpp"

namespace ax::math {

math::vecxr DenseSolver_HouseholderQR::Solve(vecxr const& b) {
  AX_THROW_IF_FALSE(impl_.isInjective(), "The factorization has not been computed.");
  vecxr x = impl_.solve(b);
  AX_THROW_IF_FALSE(impl_.info() == Eigen::Success, "Failed to solve the linear system.");
  return x;
}

void DenseSolver_HouseholderQR::Compute() {
  impl_.compute(cached_problem_->A_);
  AX_THROW_IF_FALSE(impl_.isInjective(), "The factorization has not been computed.");
}
}  // namespace ax::math
