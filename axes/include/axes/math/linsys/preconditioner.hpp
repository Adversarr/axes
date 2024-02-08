#pragma once
#include <Eigen/IterativeLinearSolvers>

#include "axes/utils/opt.hpp"
#include "common.hpp"

namespace ax::math {

enum PreconditionerKind : idx { kIdentity, kDiagonal, kIncompleteCholesky, kIncompleteLU };

class PreconditionerBase {
public:
  static utils::uptr<PreconditionerBase> Create(PreconditionerKind kind);

  virtual ~PreconditionerBase() = default;
  virtual Status Analyse(LinsysProblem_Sparse const &problem) = 0;
  virtual vecxr Solve(vecxr const &b, vecxr const &x0) = 0;
};

}  // namespace ax::math

#include "axes/utils/enum_refl.hpp"

AX_ENUM_REFL_BEGIN(ax::math::PreconditionerKind)
AX_ENUM_STATE(kIdentity, Identity)
AX_ENUM_STATE(kDiagonal, Diagonal)
AX_ENUM_STATE(kIncompleteCholesky, IncompleteCholesky)
AX_ENUM_STATE(kIncompleteLU, IncompleteLU)
AX_ENUM_REFL_END();
