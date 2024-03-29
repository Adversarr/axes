
#include "axes/math/linsys/dense/JacobiSVD.hpp"

#include "axes/utils/status.hpp"

namespace ax::math {

LinsysSolveResult DenseSolver_JacobiSVD::Solve(vecxr const& b, vecxr const&) {
  vecxr x = impl_.solve(b);
  return LinsysSolveResultImpl{std::move(x)};
}

Status DenseSolver_JacobiSVD::Analyse(problem_t const& problem) {
  impl_.compute(problem.A_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  AX_RETURN_OK();
}

}  // namespace ax::math
