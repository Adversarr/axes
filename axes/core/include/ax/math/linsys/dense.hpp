#pragma once
#include "ax/utils/enum_refl.hpp"
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
class DenseSolverBase : public LinsysSolverBase<LinsysProblem_Dense> {
public:
  static UPtr<DenseSolverBase> Create(DenseSolverKind kind);

private:
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
