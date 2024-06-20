#pragma once
#include "ax/utils/enum_refl.hpp"
#include "solver_base.hpp"
namespace ax::math {

/****************************** Embedded Solver Kinds ******************************/
BOOST_DEFINE_FIXED_ENUM_CLASS(DenseSolverKind, idx,
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
  kBDCSVD);

/****************************** Implement ******************************/
class DenseSolverBase : public LinsysSolverBase<LinsysProblem_Dense> {
public:
  static UPtr<DenseSolverBase> Create(DenseSolverKind kind);

  virtual DenseSolverKind GetKind() const = 0;
private:
};

}  // namespace ax::math

