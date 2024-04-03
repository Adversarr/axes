#include "ax/math/linsys/dense/PartialPivLU.hpp"

#include "ax/utils/status.hpp"

namespace ax::math {

LinsysSolveResult DenseSolver_PartialPivLU::Solve(vecxr const& b, vecxr const&) {
  static bool logged = false;
  if (!logged) {
   AX_LOG(WARNING) << "This method always your matrix is invertible. If you are not sure, use "
                    "FullPivLU instead.";
    logged = true;
  }
  vecxr x = impl_.solve(b);
  return LinsysSolveResultImpl{std::move(x)};
}

Status DenseSolver_PartialPivLU::Analyse(problem_t const& problem) {
  impl_.compute(problem.A_);
  static bool logged = false;
  if (!logged) {
   AX_LOG(WARNING) << "This method always your matrix is invertible. If you are not sure, use "
                    "FullPivLU instead.";
    logged = true;
  }
  AX_RETURN_OK();
}
}  // namespace ax::math
