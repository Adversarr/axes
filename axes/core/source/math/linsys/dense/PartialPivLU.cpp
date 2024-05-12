#include "ax/math/linsys/dense/PartialPivLU.hpp"

#include "ax/core/echo.hpp"
#include "ax/utils/status.hpp"

namespace ax::math {

LinsysSolveResult DenseSolver_PartialPivLU::Solve(vecxr const& b, vecxr const&) {
  AX_LOG_FIRST_N(WARNING, 1) << "This method always your matrix is invertible. If you are not sure, use "
                    "FullPivLU instead.";
   
  vecxr x = impl_.solve(b);
  return LinsysSolveResult{std::move(x)};
}

void DenseSolver_PartialPivLU::Analyse(problem_t const& problem) {
  impl_.compute(problem.A_);
  AX_LOG_FIRST_N(WARNING, 1) << "This method always your matrix is invertible. If you are not sure, use "
                                "FullPivLU instead.";
}
}  // namespace ax::math
