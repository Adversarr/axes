#include "ax/math/linsys/dense/FullPivLU.hpp"

namespace ax::math {

math::RealVectorX DenseSolver_FullPivLU::Solve(RealVectorX const& b) {
  AX_THROW_IF_FALSE(impl_.isInjective(), "The factorization has not been computed.");
  RealVectorX x = impl_.solve(b);
  return x;
}

void DenseSolver_FullPivLU::Compute() {
  impl_.compute(cached_problem_->A_);
  AX_THROW_IF_FALSE(impl_.isInjective(), "The factorization has not been computed.");
}

}  // namespace ax::math
