#pragma once
#include "ax/math/common.hpp"
#include "ax/math/sparse.hpp"
#include "ax/utils/common.hpp"
#include "ax/utils/enum_refl.hpp"
#include "ax/utils/opt.hpp"
namespace ax::optim {

BOOST_DEFINE_FIXED_ENUM_CLASS(SpsdModificationKind, Index,
    kEigenvalue,
    kCholesky,
    kIdentity,
    kDiagonal);

class SpsdModificationBase : public utils::Tunable {
public:
  virtual ~SpsdModificationBase() = default;
  std::unique_ptr<SpsdModificationBase> Create(SpsdModificationKind kind);

  virtual math::RealMatrixX Modify(math::RealMatrixX const& A) = 0;

  virtual math::RealSparseMatrix Modify(math::RealSparseMatrix const& A) = 0;
};

}  // namespace ax::optim
