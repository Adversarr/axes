#pragma once
#include "axes/utils/enum_refl.hpp"
#include "solver_base.hpp"
namespace ax::math {

/****************************** Embedded Solver Kinds ******************************/
enum class LinsysSolverKind : int {
  // SPSD
  kLDLT,
  kLLT,

  // LU Solvers
  kPartialLU,
  kFullLU,

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
class DenseSolver : DenseLinsysSolverBase {
public:
  virtual result_type Solve(vecxr const& b, vecxr const& init_guess, utils::Opt const& options);

private:
};

}  // namespace ax::math

AX_ENUM_REFL_BEGIN(ax::math::LinsysSolverKind)
AX_ENUM_STATE(kLLT, LLT)
AX_ENUM_STATE(kLDLT, LDLT)
AX_ENUM_STATE(kPartialLU, PartialLU)
AX_ENUM_STATE(kFullLU, FullLU)
AX_ENUM_STATE(kHouseholderQR, HouseholderQR)
AX_ENUM_STATE(kColPivHouseholderQR, ColPivHouseholderQR)
AX_ENUM_STATE(kFullPivHouseHolderQR, FullPivHouseHolderQR)
AX_ENUM_STATE(kCompleteOrthognalDecomposition, CompleteOrthognalDecomposition)
AX_ENUM_STATE(kJacobiSVD, JacobiSVD)
AX_ENUM_STATE(kBDCSVD, BDCSVD)
AX_ENUM_REFL_END();
