#include "ax/math/linsys/dense/PartialPivLU.hpp"

#include "ax/core/logging.hpp"

namespace ax::math {

math::RealVectorX DenseSolver_PartialPivLU::Solve(RealVectorX const& b) {
  // AX_LOG_FIRST_N(WARNING, 1)
  //     << "This method always assume your matrix is invertible. If you are not sure, use "
  //        "FullPivLU instead.";
  RealVectorX x = impl_.solve(b);
  return x;
}

void DenseSolver_PartialPivLU::Compute() {
  impl_.compute(cached_problem_->A_);
  // AX_LOG_FIRST_N(WARNING, 1)
  //     << "This method always your matrix is invertible. If you are not sure, use "
  //        "FullPivLU instead.";
}
}  // namespace ax::math
