#include "axes/math/linsys/sparse.hpp"

#include "axes/math/linsys/sparse/BiCGSTAB.hpp"
#include "axes/math/linsys/sparse/ConjugateGradient.hpp"
#include "axes/math/linsys/sparse/LDLT.hpp"
#include "axes/math/linsys/sparse/LLT.hpp"
#include "axes/math/linsys/sparse/LU.hpp"
#include "axes/math/linsys/sparse/LeastSquaresConjugateGradient.hpp"
#include "axes/math/linsys/sparse/QR.hpp"

namespace ax::math {

UPtr<SparseSolverBase> SparseSolverBase::Create(SparseSolverKind kind) {
  switch (kind) {
    case SparseSolverKind::kLDLT:
      return std::make_unique<SparseSolver_LDLT>();
    case SparseSolverKind::kLLT:
      return std::make_unique<SparseSolver_LLT>();
    case SparseSolverKind::kLU:
      return std::make_unique<SparseSolver_LU>();
    case SparseSolverKind::kQR:
      return std::make_unique<SparseSolver_QR>();
    case SparseSolverKind::kConjugateGradient:
      return std::make_unique<SparseSolver_ConjugateGradient>();
    case SparseSolverKind::kLeastSquaresConjugateGradient:
      return std::make_unique<SparseSolver_LeastSquaresConjugateGradient>();
    case SparseSolverKind::kBiCGSTAB:
      return std::make_unique<SparseSolver_BiCGSTAB>();
    default:
      return nullptr;
  }
}

SparseSolverBase::SparseSolverBase() {}

}  // namespace ax::math
