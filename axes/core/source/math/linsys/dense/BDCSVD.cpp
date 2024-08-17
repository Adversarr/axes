#include "ax/math/linsys/dense/BDCSVD.hpp"

namespace ax::math {
math::RealVectorX DenseSolver_BDCSVD::Solve(RealVectorX const& b) {
  RealVectorX x = impl_.solve(b);
  AX_THROW_IF_FALSE(impl_.info() == Eigen::Success,
                    "BDCSVD solver failed to solve the linear system.");
  return x;
}

void DenseSolver_BDCSVD::Compute() {
  impl_.compute(cached_problem_->A_, Eigen::ComputeFullU | Eigen::ComputeFullV);
}

}  // namespace ax::math
