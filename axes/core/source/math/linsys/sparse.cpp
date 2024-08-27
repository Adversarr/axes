#include "ax/math/linsys/sparse.hpp"

#include "ax/math/linsys/sparse/BiCGSTAB.hpp"
#include "ax/math/linsys/sparse/Cholmod.hpp"
#include "ax/math/linsys/sparse/ConjugateGradient.hpp"
#include "ax/math/linsys/sparse/LDLT.hpp"
#include "ax/math/linsys/sparse/LLT.hpp"
#include "ax/math/linsys/sparse/LU.hpp"
#include "ax/math/linsys/sparse/LeastSquaresConjugateGradient.hpp"
#include "ax/math/linsys/sparse/QR.hpp"

namespace ax::math {

std::unique_ptr<SparseSolverBase> SparseSolverBase::Create(SparseSolverKind kind) {
  switch (kind) {
    case SparseSolverKind::LDLT:
      return std::make_unique<SparseSolver_LDLT>();
    case SparseSolverKind::LLT:
      return std::make_unique<SparseSolver_LLT>();
    case SparseSolverKind::LU:
      return std::make_unique<SparseSolver_LU>();
    case SparseSolverKind::QR:
      return std::make_unique<SparseSolver_QR>();
    case SparseSolverKind::ConjugateGradient:
      return std::make_unique<SparseSolver_ConjugateGradient>();
    case SparseSolverKind::LeastSquaresConjugateGradient:
      return std::make_unique<SparseSolver_LeastSquaresConjugateGradient>();
    case SparseSolverKind::BiCGSTAB:
      return std::make_unique<SparseSolver_BiCGSTAB>();

      // SECT: CHOLMOD
    case SparseSolverKind::Cholmod:
      return std::make_unique<SparseSolver_Cholmod>();
    default:
      return nullptr;
  }
}

SparseSolverBase::SparseSolverBase() = default;

SparseSolverBase& SparseSolverBase::SetProblem(std::shared_ptr<LinsysProblem_Sparse> problem) {
  cached_problem_.swap(problem);
  return *this;
}

void SparseSolverBase::Compute() {
  AX_THROW_IF_NULLPTR(cached_problem_, "SparseSolverBase: problem is not set");
  AnalyzePattern();
  Factorize();
}

SparseSolverBase& SparseSolverBase::SetProblem(RealSparseMatrix const& A) {
  return SetProblem(make_sparse_problem(A));
}
SparseSolverBase& SparseSolverBase::SetProblem(RealSparseMatrix&& A) {
  return SetProblem(make_sparse_problem(std::move(A)));
}
std::shared_ptr<LinsysProblem_Sparse> const& SparseSolverBase::GetProblem() const {
  return cached_problem_;
}
void SparseSolverBase::SetPreconditioner(std::unique_ptr<PreconditionerBase> preconditioner) {
  preconditioner_ = std::move(preconditioner);
}
}  // namespace ax::math
