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
    case DenseSolverKind::LDLT:
      return std::make_unique<DenseSolver_LDLT>();
    case DenseSolverKind::LLT:
      return std::make_unique<DenseSolver_LLT>();
    case DenseSolverKind::PartialPivLU:
      return std::make_unique<DenseSolver_PartialPivLU>();
    case DenseSolverKind::FullPivLU:
      return std::make_unique<DenseSolver_FullPivLU>();
    case DenseSolverKind::HouseholderQR:
      return std::make_unique<DenseSolver_HouseholderQR>();
    case DenseSolverKind::ColPivHouseholderQR:
      return std::make_unique<DenseSolver_ColPivHouseholderQR>();
    case DenseSolverKind::FullPivHouseHolderQR:
      return std::make_unique<DenseSolver_FullPivHouseHolderQR>();
    case DenseSolverKind::CompleteOrthognalDecomposition:
      return std::make_unique<DenseSolver_CompleteOrthognalDecomposition>();
    case DenseSolverKind::JacobiSVD:
      return std::make_unique<DenseSolver_JacobiSVD>();
    case DenseSolverKind::BDCSVD:
      return std::make_unique<DenseSolver_BDCSVD>();
    default:
      return nullptr;
  }
}

}  // namespace ax::math
