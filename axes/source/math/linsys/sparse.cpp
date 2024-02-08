#include "axes/math/linsys/sparse.hpp"

#include "axes/math/linsys/sparse/BiCGSTAB.hpp"
#include "axes/math/linsys/sparse/ConjugateGradient.hpp"
#include "axes/math/linsys/sparse/LDLT.hpp"
#include "axes/math/linsys/sparse/LLT.hpp"
#include "axes/math/linsys/sparse/LU.hpp"
#include "axes/math/linsys/sparse/LeastSquaresConjugateGradient.hpp"
#include "axes/math/linsys/sparse/QR.hpp"

namespace ax::math {

utils::uptr<SparseSolverBase> SparseSolverBase::Create(SparseSolverKind kind) {
  switch (kind) {
    case kLDLT:
      return std::make_unique<SparseSolver_LDLT>();
    case kLLT:
      return std::make_unique<SparseSolver_LLT>();
    case kLU:
      return std::make_unique<SparseSolver_LU>();
    case kQR:
      return std::make_unique<SparseSolver_QR>();
    case kConjugateGradient:
      return std::make_unique<SparseSolver_ConjugateGradient>();
    case kLeastSquaresConjugateGradient:
      return std::make_unique<SparseSolver_LeastSquaresConjugateGradient>();
    case kBiCGSTAB:
      return std::make_unique<SparseSolver_BiCGSTAB>();
    default:
      return nullptr;
  }
}

}  // namespace ax::math
