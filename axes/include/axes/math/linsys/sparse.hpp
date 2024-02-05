#pragma once
#include "axes/math/linsys/solver_base.hpp"
#include "axes/utils/opt.hpp"
#include "common.hpp"
#include <Eigen/IterativeLinearSolvers>

namespace ax::math {

enum SparseSolverKind : idx {
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
  static utils::uptr<SparseSolverBase> Create(SparseSolverKind kind);
  virtual ~SparseSolverBase() = default;
};

class SparseSolver_LDLT : public SparseSolverBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem, utils::Opt const &options) override;

  result_type Solve(vecxr const &b, vecxr const &x0, utils::Opt const &options) override;
};

class SparseSolver_LLT : public SparseSolverBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem, utils::Opt const &options) override;

  result_type Solve(vecxr const &b, vecxr const &x0, utils::Opt const &options) override;
};

class SparseSolver_LU : public SparseSolverBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem, utils::Opt const &options) override;

  result_type Solve(vecxr const &b, vecxr const &x0, utils::Opt const &options) override;
};

class SparseSolver_QR : public SparseSolverBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem, utils::Opt const &options) override;

  result_type Solve(vecxr const &b, vecxr const &x0, utils::Opt const &options) override;
};

class SparseSolver_ConjugateGradient : public SparseSolverBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem, utils::Opt const &options) override;

  result_type Solve(vecxr const &b, vecxr const &x0, utils::Opt const &options) override;
};

class SparseSolver_LeastSquaresConjugateGradient : public SparseSolverBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem, utils::Opt const &options) override;

  result_type Solve(vecxr const &b, vecxr const &x0, utils::Opt const &options) override;
};

class SparseSolver_BiCGSTAB : public SparseSolverBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem, utils::Opt const &options) override;

  result_type Solve(vecxr const &b, vecxr const &x0, utils::Opt const &options) override;
};

}  // namespace ax::math

#include "axes/utils/enum_refl.hpp"

AX_ENUM_REFL_BEGIN(ax::math::SparseSolverKind)
AX_ENUM_STATE(kLLT, LLT)
AX_ENUM_STATE(kLDLT, LDLT)
AX_ENUM_STATE(kLU, LU)
AX_ENUM_STATE(kQR, QR)
AX_ENUM_STATE(kConjugateGradient, ConjugateGradient)
AX_ENUM_STATE(kLeastSquaresConjugateGradient, LeastSquaresConjugateGradient)
AX_ENUM_STATE(kBiCGSTAB, BiCGSTAB)
AX_ENUM_REFL_END();
