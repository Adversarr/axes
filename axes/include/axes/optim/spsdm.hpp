#pragma once
#include "axes/math/common.hpp"
#include "axes/utils/common.hpp"
#include "axes/utils/opt.hpp"

namespace ax::optim {

enum class SpsdModificationKind : idx { kEigenvalue, kCholesky, kIdentity };

class SpsdModificationBase : public utils::Tunable {
public:
  virtual ~SpsdModificationBase() = default;
  UPtr<SpsdModificationBase> Create(SpsdModificationKind kind);

  virtual StatusOr<math::matxxr> Modify(math::matxxr const& A)
      = 0;
};

}  // namespace ax::optim

/************************* SECT: Reflection *************************/
#include "axes/utils/enum_refl.hpp"
AX_ENUM_REFL_BEGIN(ax::optim::SpsdModificationKind)
AX_ENUM_STATE(kEigenvalue, Eigenvalue)
AX_ENUM_STATE(kCholesky, Cholesky)
AX_ENUM_STATE(kIdentity, Identity)
AX_ENUM_REFL_END();
