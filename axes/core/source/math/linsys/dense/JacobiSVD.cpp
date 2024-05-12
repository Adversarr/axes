
#include "ax/math/linsys/dense/JacobiSVD.hpp"

#include "ax/utils/status.hpp"

namespace ax::math {

LinsysSolveResult DenseSolver_JacobiSVD::Solve(vecxr const& b, vecxr const&) {
  vecxr x = impl_.solve(b);
  return LinsysSolveResult{std::move(x)};
}

void DenseSolver_JacobiSVD::Analyse(problem_t const& problem) {
  impl_.compute(problem.A_, Eigen::ComputeFullU | Eigen::ComputeFullV);
}

}  // namespace ax::math
