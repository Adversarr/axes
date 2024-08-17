#include "ax/math/linsys/dense/HouseholderQR.hpp"

namespace ax::math {

math::RealVectorX DenseSolver_HouseholderQR::Solve(RealVectorX const& b) {
  AX_THROW_IF_FALSE(impl_.isInjective(), "The factorization has not been computed.");
  RealVectorX x = impl_.solve(b);
  AX_THROW_IF_FALSE(impl_.info() == Eigen::Success, "Failed to solve the linear system.");
  return x;
}

void DenseSolver_HouseholderQR::Compute() {
  impl_.compute(cached_problem_->A_);
  AX_THROW_IF_FALSE(impl_.isInjective(), "The factorization has not been computed.");
}
}  // namespace ax::math
