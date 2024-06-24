#include "ax/math/linsys/dense/BDCSVD.hpp"

namespace ax::math {
math::vecxr DenseSolver_BDCSVD::Solve(vecxr const& b) {
  vecxr x = impl_.solve(b);
  AX_THROW_IF_FALSE(impl_.info() == Eigen::Success,
                    "BDCSVD solver failed to solve the linear system.");
  return x;
}

void DenseSolver_BDCSVD::Compute() {
  impl_.compute(cached_problem_->A_, Eigen::ComputeFullU | Eigen::ComputeFullV);
}

}  // namespace ax::math
