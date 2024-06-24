#include "ax/math/linsys/sparse.hpp"

#include "ax/math/linsys/sparse/BiCGSTAB.hpp"
#include "ax/math/linsys/sparse/ConjugateGradient.hpp"
#include "ax/math/linsys/sparse/LDLT.hpp"
#include "ax/math/linsys/sparse/LLT.hpp"
#include "ax/math/linsys/sparse/LU.hpp"
#include "ax/math/linsys/sparse/LeastSquaresConjugateGradient.hpp"
#include "ax/math/linsys/sparse/QR.hpp"

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

SparseSolverBase& SparseSolverBase::SetProblem(SPtr<LinsysProblem_Sparse> problem) {
  cached_problem_ = std::move(problem);
  return *this;
}

void SparseSolverBase::Compute() {
  AX_THROW_IF_NULL(cached_problem_, "SparseSolverBase: problem is not set");
  AnalyzePattern();
  Factorize();
}

SparseSolverBase& SparseSolverBase::SetProblem(spmatr const& A) {
  return SetProblem(make_sparse_problem(A));
}
SparseSolverBase& SparseSolverBase::SetProblem(spmatr&& A) {
  return SetProblem(make_sparse_problem(std::move(A)));
}
SPtr<LinsysProblem_Sparse> const& SparseSolverBase::GetProblem() const { return cached_problem_; }
void SparseSolverBase::SetPreconditioner(UPtr<PreconditionerBase> preconditioner) {
  preconditioner_ = std::move(preconditioner);
}
}  // namespace ax::math
