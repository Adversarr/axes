#include "axes/math/linsys/preconditioner.hpp"
#include "axes/math/linsys/preconditioner/Diagonal.hpp"
#include "axes/math/linsys/preconditioner/Identity.hpp"
#include "axes/math/linsys/preconditioner/IncompleteCholesky.hpp"
#include "axes/math/linsys/preconditioner/IncompleteLU.hpp"

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

}  // namespace ax::math