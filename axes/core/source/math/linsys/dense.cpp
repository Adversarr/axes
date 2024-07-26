#include "ax/math/linsys/dense.hpp"

#include "ax/math/linsys/dense/BDCSVD.hpp"
#include "ax/math/linsys/dense/ColPivHouseholderQR.hpp"
#include "ax/math/linsys/dense/CompleteOrthognalDecomposition.hpp"
#include "ax/math/linsys/dense/FullPivHouseholderQR.hpp"
#include "ax/math/linsys/dense/FullPivLU.hpp"
#include "ax/math/linsys/dense/HouseholderQR.hpp"
#include "ax/math/linsys/dense/JacobiSVD.hpp"
#include "ax/math/linsys/dense/LDLT.hpp"
#include "ax/math/linsys/dense/LLT.hpp"
#include "ax/math/linsys/dense/PartialPivLU.hpp"
namespace ax::math {

std::unique_ptr<DenseSolverBase> DenseSolverBase::Create(DenseSolverKind kind) {
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
