#include "axes/math/linsys/preconditioner.hpp"

#include "axes/utils/status.hpp"

namespace ax::math {

utils::uptr<PreconditionerBase> PreconditionerBase::Create(PreconditionerKind kind) {
  switch (kind) {
    case kIdentity:
      return std::make_unique<PreconditionerIdentity>();
    case kDiagonal:
      return std::make_unique<PreconditionerDiagonal>();
    case kIncompleteCholesky:
      return std::make_unique<PreconditionerIncompleteCholesky>();
    case kIncompleteLU:
      return std::make_unique<PreconditionerIncompleteLU>();
    default:
      return nullptr;
  }
}

Status PreconditionerIdentity::Analyse(LinsysProblem_Sparse const &) { AX_RETURN_OK(); }

vecxr PreconditionerIdentity::Solve(vecxr const &b, vecxr const &) { return b; }

Status PreconditionerDiagonal::Analyse(LinsysProblem_Sparse const &problem) {
  impl_.compute(problem.A_);
  if (!(impl_.info() == Eigen::Success)) {
    return utils::FailedPreconditionError("The factorization has not been computed.");
  }
  AX_RETURN_OK();
}

vecxr PreconditionerDiagonal::Solve(vecxr const &b, vecxr const &) { return impl_.solve(b); }

Status PreconditionerIncompleteCholesky::Analyse(LinsysProblem_Sparse const &problem) {
  impl_.compute(problem.A_);
  if (!(impl_.info() == Eigen::Success)) {
    return utils::FailedPreconditionError("The factorization has not been computed.");
  }
  AX_RETURN_OK();
}

vecxr PreconditionerIncompleteCholesky::Solve(vecxr const &b, vecxr const &) {
  return impl_.solve(b);
}

Status PreconditionerIncompleteLU::Analyse(LinsysProblem_Sparse const &problem) {
  impl_.compute(problem.A_);
  if (!(impl_.info() == Eigen::Success)) {
    return utils::FailedPreconditionError("The factorization has not been computed.");
  }
  AX_RETURN_OK();
}

vecxr PreconditionerIncompleteLU::Solve(vecxr const &b, vecxr const &) { return impl_.solve(b); }

}  // namespace ax::math