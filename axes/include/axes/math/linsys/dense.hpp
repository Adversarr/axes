#pragma once
#include "axes/utils/enum_refl.hpp"
#include "solver_base.hpp"
namespace ax::math {

/****************************** Embedded Solver Kinds ******************************/
enum class DenseSolverKind : int {
  // SPSD
  kLDLT,
  kLLT,

  // LU Solvers
  kPartialPivLU,
  kFullPivLU,

  // QR-based Solvers
  kHouseholderQR,
  kColPivHouseholderQR,
  kFullPivHouseHolderQR,
  kCompleteOrthognalDecomposition,

  // SVD-based Solvers
  kJacobiSVD,
  kBDCSVD,
};

/****************************** Implement ******************************/
class DenseSolver : public LinsysSolverBase<LinsysProblem_Dense> {
public:
  static utils::uptr<DenseSolver> Create(DenseSolverKind kind);

private:
};

class DenseSolver_LLT : public DenseSolver {
public:
  result_type Solve(vecxr const& b, vecxr const& init_guess, utils::Opt const& options);
  Status Analyse(problem_t const& problem, utils::Opt const& options);

private:
  Eigen::LLT<matxxr> impl_;
};

class DenseSolver_LDLT : public DenseSolver {
public:
  result_type Solve(vecxr const& b, vecxr const& init_guess, utils::Opt const& options);
  Status Analyse(problem_t const& problem, utils::Opt const& options);

private:
  Eigen::LDLT<matxxr> impl_;
};

class DenseSolver_PartialPivLU : public DenseSolver {
public:
  result_type Solve(vecxr const& b, vecxr const& init_guess, utils::Opt const& options);
  Status Analyse(problem_t const& problem, utils::Opt const& options);

private:
  Eigen::PartialPivLU<matxxr> impl_;
};

class DenseSolver_FullPivLU : public DenseSolver {
public:
  result_type Solve(vecxr const& b, vecxr const& init_guess, utils::Opt const& options);
  Status Analyse(problem_t const& problem, utils::Opt const& options);

private:
  Eigen::FullPivLU<matxxr> impl_;
};

class DenseSolver_HouseholderQR : public DenseSolver {
public:
  result_type Solve(vecxr const& b, vecxr const& init_guess, utils::Opt const& options);
  Status Analyse(problem_t const& problem, utils::Opt const& options);

private:
  Eigen::ColPivHouseholderQR<matxxr> impl_;
};

class DenseSolver_ColPivHouseholderQR : public DenseSolver {
public:
  result_type Solve(vecxr const& b, vecxr const& init_guess, utils::Opt const& options);
  Status Analyse(problem_t const& problem, utils::Opt const& options);

private:
  Eigen::ColPivHouseholderQR<matxxr> impl_;
};

class DenseSolver_FullPivHouseHolderQR : public DenseSolver {
public:
  result_type Solve(vecxr const& b, vecxr const& init_guess, utils::Opt const& options);
  Status Analyse(problem_t const& problem, utils::Opt const& options);

private:
  Eigen::FullPivHouseholderQR<matxxr> impl_;
};

class DenseSolver_CompleteOrthognalDecomposition : public DenseSolver {
public:
  result_type Solve(vecxr const& b, vecxr const& init_guess, utils::Opt const& options);
  Status Analyse(problem_t const& problem, utils::Opt const& options);

private:
  Eigen::CompleteOrthogonalDecomposition<matxxr> impl_;
};

class DenseSolver_JacobiSVD : public DenseSolver {
public:
  result_type Solve(vecxr const& b, vecxr const& init_guess, utils::Opt const& options);
  Status Analyse(problem_t const& problem, utils::Opt const& options);

private:
  Eigen::JacobiSVD<matxxr> impl_;
};

class DenseSolver_BDCSVD : public DenseSolver {
public:
  result_type Solve(vecxr const& b, vecxr const& init_guess, utils::Opt const& options);
  Status Analyse(problem_t const& problem, utils::Opt const& options);

private:
  Eigen::BDCSVD<matxxr> impl_;
};

}  // namespace ax::math
AX_ENUM_REFL_BEGIN(ax::math::DenseSolverKind)
  AX_ENUM_STATE(kLLT, LLT)
  AX_ENUM_STATE(kLDLT, LDLT)
  AX_ENUM_STATE(kPartialPivLU, PartialPivLU)
  AX_ENUM_STATE(kFullPivLU, FullPivLU)
  AX_ENUM_STATE(kHouseholderQR, HouseholderQR)
  AX_ENUM_STATE(kColPivHouseholderQR, ColPivHouseholderQR)
  AX_ENUM_STATE(kFullPivHouseHolderQR, FullPivHouseHolderQR)
  AX_ENUM_STATE(kCompleteOrthognalDecomposition, CompleteOrthognalDecomposition)
  AX_ENUM_STATE(kJacobiSVD, JacobiSVD)
  AX_ENUM_STATE(kBDCSVD, BDCSVD)
AX_ENUM_REFL_END();

