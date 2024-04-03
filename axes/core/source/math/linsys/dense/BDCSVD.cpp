#include "ax/math/linsys/dense/BDCSVD.hpp"

#include "ax/utils/status.hpp"

namespace ax::math {
LinsysSolveResult DenseSolver_BDCSVD::Solve(vecxr const& b, vecxr const&) {
  vecxr x = impl_.solve(b);
  return LinsysSolveResultImpl{std::move(x)};
}

Status DenseSolver_BDCSVD::Analyse(problem_t const& problem) {
  impl_.compute(problem.A_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  AX_RETURN_OK();
}

}  // namespace ax::math
