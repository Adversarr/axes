#pragma once
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>

#include "ax/math/linsys/preconditioner.hpp"
#include "ax/math/linsys/solver_base.hpp"
#include "common.hpp"


#include "ax/utils/enum_refl.hpp"

namespace ax::math {

BOOST_DEFINE_FIXED_ENUM_CLASS(SparseSolverKind, idx,
  kLDLT,
  kLLT,
  kLU,
  kQR,
  kConjugateGradient,
  kLeastSquaresConjugateGradient,
  kBiCGSTAB)

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
