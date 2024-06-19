#include "ax/math/linsys/preconditioner.hpp"

#include "ax/math/linsys/preconditioner/Diagonal.hpp"
#include "ax/math/linsys/preconditioner/Identity.hpp"
#include "ax/math/linsys/preconditioner/IncompleteCholesky.hpp"
#include "ax/math/linsys/preconditioner/IncompleteLU.hpp"
#include "ax/utils/status.hpp"

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

}  // namespace ax::math