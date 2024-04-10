#pragma once
#include <Eigen/IterativeLinearSolvers>

#include "ax/utils/enum_refl.hpp"
#include "ax/utils/opt.hpp"
#include "common.hpp"

namespace ax::math {

BOOST_DEFINE_FIXED_ENUM_CLASS(PreconditionerKind, idx,
  kIdentity,
  kDiagonal,
  kIncompleteCholesky,
  kIncompleteLU);

class PreconditionerBase : public utils::Tunable {
public:
  static UPtr<PreconditionerBase> Create(PreconditionerKind kind);

  virtual ~PreconditionerBase() = default;
  virtual Status Analyse(LinsysProblem_Sparse const &problem) = 0;
  virtual vecxr Solve(vecxr const &b, vecxr const &x0) = 0;

  virtual PreconditionerKind Kind() const = 0;
};

}  // namespace ax::math

