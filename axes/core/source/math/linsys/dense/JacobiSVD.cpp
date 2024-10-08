#include "ax/math/linsys/dense/JacobiSVD.hpp"

namespace ax::math {

math::RealVectorX DenseSolver_JacobiSVD::Solve(const math::RealVectorX& b) {
  RealVectorX x = impl_.solve(b);
  AX_THROW_IF_FALSE(impl_.info() == Eigen::Success, "Eigen::JacobiSVD: solve failed.");
  return x;
}

void DenseSolver_JacobiSVD::Compute() {
  impl_.compute(cached_problem_->A_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  AX_THROW_IF_FALSE(impl_.info() == Eigen::Success, "Eigen::JacobiSVD: compute failed");
}

}  // namespace ax::math
