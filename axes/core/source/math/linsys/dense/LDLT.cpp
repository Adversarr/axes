#include "ax/math/linsys/dense/LDLT.hpp"

namespace ax::math {

math::vecxr DenseSolver_LDLT::Solve(const math::vecxr &b) {
  vecxr x = impl_.solve(b);
  AX_THROW_IF_FALSE(impl_.info() == Eigen::Success, "Eigen::LDLT: Solve failed.");
  return x;
}

void DenseSolver_LDLT::Compute() {
  impl_.compute(cached_problem_->A_);
  AX_THROW_IF_FALSE(impl_.info() == Eigen::Success, "Eigen::LDLT: The factorization has not been computed.");
  AX_THROW_IF_FALSE(impl_.isPositive(), "Eigen::LDLT: The matrix is not positive definite.");
}

}  // namespace ax::math
