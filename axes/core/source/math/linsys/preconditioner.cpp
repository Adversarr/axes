#include "ax/math/linsys/preconditioner.hpp"

#include "ax/math/linsys/preconditioner/Diagonal.hpp"
#include "ax/math/linsys/preconditioner/Identity.hpp"
#include "ax/math/linsys/preconditioner/IncompleteCholesky.hpp"
#include "ax/math/linsys/preconditioner/IncompleteLU.hpp"

namespace ax::math {

UPtr<PreconditionerBase> PreconditionerBase::Create(PreconditionerKind kind) {
  switch (kind) {
    case ax::math::PreconditionerKind::kIdentity:
      return std::make_unique<Preconditioner_Identity>();
    case ax::math::PreconditionerKind::kDiagonal:
      return std::make_unique<Preconditioner_Diagonal>();
    case ax::math::PreconditionerKind::kIncompleteCholesky:
      return std::make_unique<Preconditioner_IncompleteCholesky>();
    case ax::math::PreconditionerKind::kIncompleteLU:
      return std::make_unique<Preconditioner_IncompleteLU>();
    default:
      return nullptr;
  }
}

PreconditionerBase& PreconditionerBase::SetProblem(SPtr<LinsysProblem_Sparse> problem) {
  cached_problem_ = std::move(problem);
  return *this;
}

void PreconditionerBase::Compute() {
  AX_THROW_IF_NULL(cached_problem_, "PreconditionerBase::Compute: problem is not set");

  AnalyzePattern();
  Factorize();
}

}  // namespace ax::math
