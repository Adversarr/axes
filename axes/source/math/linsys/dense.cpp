#include "axes/math/linsys/dense.hpp"

#include "axes/math/linsys/dense/BDCSVD.hpp"
#include "axes/math/linsys/dense/ColPivHouseholderQR.hpp"
#include "axes/math/linsys/dense/CompleteOrthognalDecomposition.hpp"
#include "axes/math/linsys/dense/FullPivHouseholderQR.hpp"
#include "axes/math/linsys/dense/FullPivLU.hpp"
#include "axes/math/linsys/dense/HouseholderQR.hpp"
#include "axes/math/linsys/dense/JacobiSVD.hpp"
#include "axes/math/linsys/dense/LDLT.hpp"
#include "axes/math/linsys/dense/LLT.hpp"
#include "axes/math/linsys/dense/PartialPivLU.hpp"
namespace ax::math {

utils::uptr<DenseSolverBase> DenseSolverBase::Create(DenseSolverKind kind) {
  switch (kind) {
    case DenseSolverKind::kLDLT:
      return std::make_unique<DenseSolver_LDLT>();
    case DenseSolverKind::kLLT:
      return std::make_unique<DenseSolver_LLT>();
    case DenseSolverKind::kPartialPivLU:
      return std::make_unique<DenseSolver_PartialPivLU>();
    case DenseSolverKind::kFullPivLU:
      return std::make_unique<DenseSolver_FullPivLU>();
    case DenseSolverKind::kHouseholderQR:
      return std::make_unique<DenseSolver_HouseholderQR>();
    case DenseSolverKind::kColPivHouseholderQR:
      return std::make_unique<DenseSolver_ColPivHouseholderQR>();
    case DenseSolverKind::kFullPivHouseHolderQR:
      return std::make_unique<DenseSolver_FullPivHouseHolderQR>();
    case DenseSolverKind::kCompleteOrthognalDecomposition:
      return std::make_unique<DenseSolver_CompleteOrthognalDecomposition>();
    case DenseSolverKind::kJacobiSVD:
      return std::make_unique<DenseSolver_JacobiSVD>();
    case DenseSolverKind::kBDCSVD:
      return std::make_unique<DenseSolver_BDCSVD>();
    default:
      return nullptr;
  }
}

}  // namespace ax::math
