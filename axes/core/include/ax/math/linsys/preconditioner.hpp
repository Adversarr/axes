#pragma once
#include <Eigen/IterativeLinearSolvers>

#include "ax/utils/enum_refl.hpp"
#include "ax/utils/opt.hpp"
#include "common.hpp"

namespace ax::math {

AX_DEFINE_ENUM_CLASS(PreconditionerKind, Identity, Diagonal, IncompleteCholesky, IncompleteLU, FromLambda);

class PreconditionerBase : public utils::Tunable {
public:
  static std::unique_ptr<PreconditionerBase> Create(PreconditionerKind kind);
  virtual ~PreconditionerBase() = default;

  virtual void AnalyzePattern() = 0;
  virtual void Factorize() = 0;
  virtual RealMatrixX Solve(RealMatrixX const& b) = 0;

  PreconditionerBase& SetProblem(std::shared_ptr<LinsysProblem_Sparse> problem);

  virtual PreconditionerKind GetKind() const = 0;

  void Compute();

protected:
  std::shared_ptr<LinsysProblem_Sparse> cached_problem_{nullptr};
};

}  // namespace ax::math
