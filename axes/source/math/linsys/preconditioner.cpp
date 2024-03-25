#include "axes/math/linsys/preconditioner.hpp"

#include "axes/math/linsys/preconditioner/Diagonal.hpp"
#include "axes/math/linsys/preconditioner/Identity.hpp"
#include "axes/math/linsys/preconditioner/IncompleteCholesky.hpp"
#include "axes/math/linsys/preconditioner/IncompleteLU.hpp"
#include "axes/utils/status.hpp"

namespace ax::math {

UPtr<PreconditionerBase> PreconditionerBase::Create(PreconditionerKind kind) {
  switch (kind) {
    case ax::math::PreconditionerKind::kIdentity:
      return std::make_unique<PreconditionerIdentity>();
    case ax::math::PreconditionerKind::kDiagonal:
      return std::make_unique<PreconditionerDiagonal>();
    case ax::math::PreconditionerKind::kIncompleteCholesky:
      return std::make_unique<PreconditionerIncompleteCholesky>();
    case ax::math::PreconditionerKind::kIncompleteLU:
      return std::make_unique<PreconditionerIncompleteLU>();
    default:
      return nullptr;
  }
}

}  // namespace ax::math