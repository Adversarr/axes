#pragma once
#include <Eigen/IterativeLinearSolvers>

#include "ax/utils/enum_refl.hpp"
#include "ax/utils/opt.hpp"
#include "common.hpp"

namespace ax::math {

BOOST_DEFINE_FIXED_ENUM_CLASS(PreconditionerKind, idx, kIdentity, kDiagonal, kIncompleteCholesky,
                              kIncompleteLU);

class PreconditionerBase : public utils::Tunable {
public:
  static UPtr<PreconditionerBase> Create(PreconditionerKind kind);
  virtual ~PreconditionerBase() = default;

  virtual void AnalyzePattern() = 0;
  virtual void Factorize() = 0;
  virtual vecxr Solve(vecxr const& b) = 0;

  PreconditionerBase& SetProblem(SPtr<LinsysProblem_Sparse> problem);

  virtual PreconditionerKind GetKind() const = 0;

  void Compute();

protected:
  SPtr<LinsysProblem_Sparse> cached_problem_{nullptr};
};

}  // namespace ax::math
