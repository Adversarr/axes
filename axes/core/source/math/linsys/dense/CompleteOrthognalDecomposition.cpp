#include "ax/math/linsys/dense/CompleteOrthognalDecomposition.hpp"

namespace ax::math {

math::RealVectorX DenseSolver_CompleteOrthognalDecomposition::Solve(RealVectorX const& b) {
  AX_THROW_IF_FALSE(impl_.isInjective(), "The factorization has not been computed.");
  RealVectorX x = impl_.solve(b);
  AX_THROW_IF_FALSE(impl_.info() == Eigen::Success, "Failed to solve the linear system.");
  return x;
}

void DenseSolver_CompleteOrthognalDecomposition::Compute() {
  impl_.compute(cached_problem_->A_);
  AX_THROW_IF_FALSE(impl_.isInjective(), "The factorization has not been computed.");
}
}  // namespace ax::math
