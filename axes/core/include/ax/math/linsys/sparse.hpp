#pragma once
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>

#include "ax/math/linsys/preconditioner.hpp"
#include "ax/math/linsys/solver_base.hpp"
#include "common.hpp"

namespace ax::math {

enum class SparseSolverKind : idx {
  // Direct
  kLDLT,
  kLLT,
  kLU,
  kQR,

  // Iterative
  kConjugateGradient,
  kLeastSquaresConjugateGradient,
  kBiCGSTAB
};

class SparseSolverBase : public LinsysSolverBase<LinsysProblem_Sparse> {
public:
  SparseSolverBase();

  static UPtr<SparseSolverBase> Create(SparseSolverKind kind);

  inline void SetPreconditioner(UPtr<PreconditionerBase> preconditioner) {
    preconditioner_ = std::move(preconditioner);
  }

  virtual ~SparseSolverBase() = default;

  virtual SparseSolverKind Kind() const = 0;

protected:
  UPtr<PreconditionerBase> preconditioner_{nullptr};
};



}  // namespace ax::math

#include "ax/utils/enum_refl.hpp"

AX_ENUM_REFL_BEGIN(ax::math::SparseSolverKind)
AX_ENUM_STATE(kLLT, LLT)
AX_ENUM_STATE(kLDLT, LDLT)
AX_ENUM_STATE(kLU, LU)
AX_ENUM_STATE(kQR, QR)
AX_ENUM_STATE(kConjugateGradient, ConjugateGradient)
AX_ENUM_STATE(kLeastSquaresConjugateGradient, LeastSquaresConjugateGradient)
AX_ENUM_STATE(kBiCGSTAB, BiCGSTAB)
AX_ENUM_REFL_END();
