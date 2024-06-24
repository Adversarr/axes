#include "ax/math/linsys/dense/PartialPivLU.hpp"

#include "ax/core/echo.hpp"
#include "ax/utils/status.hpp"

namespace ax::math {

math::vecxr DenseSolver_PartialPivLU::Solve(vecxr const& b) {
  AX_LOG_FIRST_N(WARNING, 1)
      << "This method always assume your matrix is invertible. If you are not sure, use "
         "FullPivLU instead.";
  vecxr x = impl_.solve(b);
  return x;
}

void DenseSolver_PartialPivLU::Compute() {
  impl_.compute(cached_problem_->A_);
  AX_LOG_FIRST_N(WARNING, 1)
      << "This method always your matrix is invertible. If you are not sure, use "
         "FullPivLU instead.";
}
}  // namespace ax::math
