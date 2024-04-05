#include "ax/optim/spsdm.hpp"
#include "ax/optim/spsdm/diagonal.hpp"
#include "ax/optim/spsdm/eigenvalue.hpp"
namespace ax::optim {

UPtr<SpsdModificationBase> SpsdModificationBase::Create(SpsdModificationKind kind) {
  switch (kind) {
    case SpsdModificationKind::kEigenvalue:
      return std::make_unique<EigenvalueModification>();
    case SpsdModificationKind::kCholesky:
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
