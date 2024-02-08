#include "axes/math/linsys/preconditioner/IncompleteLU.hpp"

#include "axes/utils/status.hpp"

namespace ax::math {

Status PreconditionerIncompleteLU::Analyse(LinsysProblem_Sparse const &problem) {
  impl_.compute(problem.A_);
  if (!(impl_.info() == Eigen::Success)) {
    return utils::FailedPreconditionError("The factorization has not been computed.");
  }
  AX_RETURN_OK();
}

vecxr PreconditionerIncompleteLU::Solve(vecxr const &b, vecxr const &) { return impl_.solve(b); }
}  // namespace ax::math