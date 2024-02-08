#pragma once
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>

#include "axes/math/linsys/preconditioner.hpp"
#include "axes/math/linsys/solver_base.hpp"
#include "axes/utils/opt.hpp"
#include "common.hpp"

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

  inline void SetPreconditioner(utils::uptr<PreconditionerBase> preconditioner) {
    preconditioner_ = std::move(preconditioner);
  }

  virtual ~SparseSolverBase() = default;

protected:
  utils::uptr<PreconditionerBase> preconditioner_{nullptr};
};

class SparseSolver_LDLT : public SparseSolverBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem) override;

  result_type Solve(vecxr const &b, vecxr const &x0) override;

private:
  Eigen::SimplicialLDLT<sp_matxxr> solver_;
};

class SparseSolver_LLT : public SparseSolverBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem) override;

  result_type Solve(vecxr const &b, vecxr const &x0) override;

private:
  Eigen::SimplicialLLT<sp_matxxr> solver_;
};

class SparseSolver_LU : public SparseSolverBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem) override;

  result_type Solve(vecxr const &b, vecxr const &x0) override;

private:
  Eigen::SparseLU<sp_matxxr, Eigen::COLAMDOrdering<idx>> solver_;
};

class SparseSolver_QR : public SparseSolverBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem) override;

  result_type Solve(vecxr const &b, vecxr const &x0) override;

private:
  Eigen::SparseQR<sp_matxxr, Eigen::COLAMDOrdering<idx>> solver_;
};

class SparseSolver_ConjugateGradient : public SparseSolverBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem) override;

  result_type Solve(vecxr const &b, vecxr const &x0) override;

private:
  Eigen::ConjugateGradient<sp_matxxr, Eigen::Lower | Eigen::Upper,
                           Eigen::DiagonalPreconditioner<real>>
      solver_;
};

class SparseSolver_LeastSquaresConjugateGradient : public SparseSolverBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem) override;

  result_type Solve(vecxr const &b, vecxr const &x0) override;

private:
  Eigen::LeastSquaresConjugateGradient<sp_matxxr> solver_;
};

class SparseSolver_BiCGSTAB : public SparseSolverBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem) override;

  result_type Solve(vecxr const &b, vecxr const &x0) override;

private:
  Eigen::BiCGSTAB<sp_matxxr> solver_;
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
