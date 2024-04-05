#pragma once
#include "ax/math/common.hpp"
#include "ax/math/sparse.hpp"
#include "ax/utils/common.hpp"
#include "ax/utils/opt.hpp"

namespace ax::optim {

enum class SpsdModificationKind : idx { kEigenvalue, kCholesky, kIdentity, kDiagonal };

class SpsdModificationBase : public utils::Tunable {
public:
  virtual ~SpsdModificationBase() = default;
  UPtr<SpsdModificationBase> Create(SpsdModificationKind kind);

  virtual StatusOr<math::matxxr> Modify(math::matxxr const& A) = 0;

  virtual StatusOr<math::sp_matxxr> Modify(math::sp_matxxr const& A) = 0;
};

}  // namespace ax::optim

/************************* SECT: Reflection *************************/
#include "ax/utils/enum_refl.hpp"
AX_ENUM_REFL_BEGIN(ax::optim::SpsdModificationKind)
AX_ENUM_STATE(kEigenvalue, Eigenvalue)
AX_ENUM_STATE(kCholesky, Cholesky)
AX_ENUM_STATE(kIdentity, Identity)
AX_ENUM_STATE(kDiagonal, Diagonal)
AX_ENUM_REFL_END();
