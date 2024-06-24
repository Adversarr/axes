#include "ax/math/linsys/dense/ColPivHouseholderQR.hpp"

namespace ax::math {

math::vecxr DenseSolver_ColPivHouseholderQR::Solve(vecxr const& b) {
  AX_THROW_IF_FALSE(impl_.isInjective(),
                    "Eigen::ColPivHouseholderQR: The factorization has not been computed.");
  vecxr x = impl_.solve(b);
  AX_THROW_IF_FALSE(impl_.info() == Eigen::Success,
                    "Eigen::ColPivHouseholderQR: Failed to solve the linear system.");
  return x;
}

void DenseSolver_ColPivHouseholderQR::Compute() {
  impl_.compute(cached_problem_->A_);
  AX_THROW_IF_FALSE(impl_.isInjective(),
                    "Eigen::ColPivHouseholderQR: The factorization has not been computed.");
}

}  // namespace ax::math
