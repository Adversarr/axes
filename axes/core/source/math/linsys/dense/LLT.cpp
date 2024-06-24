#include "ax/math/linsys/dense/LLT.hpp"

namespace ax::math {

math::vecxr DenseSolver_LLT::Solve(vecxr const& b) {
  vecxr x = impl_.solve(b);
  AX_THROW_IF_FALSE(impl_.info() == Eigen::Success, "Eigen::LLT: Solve failed.");
  return x;
}

void DenseSolver_LLT::Compute() {
  impl_.compute(cached_problem_->A_);
  AX_THROW_IF_FALSE(impl_.info() == Eigen::Success, "The factorization has not been computed.");
}

}  // namespace ax::math
