#include "ax/optim/spsdm.hpp"
#include "ax/optim/spsdm/diagonal.hpp"
#include "ax/optim/spsdm/eigenvalue.hpp"
namespace ax::optim {

std::unique_ptr<SpsdModificationBase> SpsdModificationBase::Create(SpsdModificationKind kind) {
  switch (kind) {
    case SpsdModificationKind::Eigenvalue:
      return std::make_unique<EigenvalueModification>();
    case SpsdModificationKind::Cholesky:
      return std::make_unique<DiagonalModification>();
    // case SpsdModificationKind::kIdentity:
    //   return std::make_unique<SpsdIdentity>();
    // case SpsdModificationKind::kDiagonal:
    //   return std::make_unique<SpsdDiagonal>();
    default:
      return nullptr;
  }
}

}  // namespace ax::optim
