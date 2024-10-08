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

std::unique_ptr<HostSparseSolverBase> HostSparseSolverBase::Create(HostSparseSolverKind kind) {
  switch (kind) {
    case HostSparseSolverKind::LDLT:
      return std::make_unique<SparseSolver_LDLT>();
    case HostSparseSolverKind::LLT:
      return std::make_unique<SparseSolver_LLT>();
    case HostSparseSolverKind::LU:
      return std::make_unique<SparseSolver_LU>();
    case HostSparseSolverKind::QR:
      return std::make_unique<SparseSolver_QR>();
    case HostSparseSolverKind::ConjugateGradient:
      return std::make_unique<SparseSolver_ConjugateGradient>();
    case HostSparseSolverKind::LeastSquaresConjugateGradient:
      return std::make_unique<SparseSolver_LeastSquaresConjugateGradient>();
    case HostSparseSolverKind::BiCGSTAB:
      return std::make_unique<SparseSolver_BiCGSTAB>();

      // SECT: CHOLMOD
    case HostSparseSolverKind::Cholmod:
      return std::make_unique<SparseSolver_Cholmod>();
    default:
      return nullptr;
  }
}

HostSparseSolverBase::HostSparseSolverBase() = default;

HostSparseSolverBase& HostSparseSolverBase::SetProblem(std::shared_ptr<LinsysProblem_Sparse> problem) {
  cached_problem_.swap(problem);
  return *this;
}

void HostSparseSolverBase::Compute() {
  AX_THROW_IF_NULLPTR(cached_problem_, "SparseSolverBase: problem is not set");
  AnalyzePattern();
  Factorize();
}

HostSparseSolverBase& HostSparseSolverBase::SetProblem(RealSparseMatrix const& A) {
  return SetProblem(make_sparse_problem(A));
}
HostSparseSolverBase& HostSparseSolverBase::SetProblem(RealSparseMatrix&& A) {
  return SetProblem(make_sparse_problem(std::move(A)));
}
std::shared_ptr<LinsysProblem_Sparse> const& HostSparseSolverBase::GetProblem() const {
  return cached_problem_;
}
void HostSparseSolverBase::SetPreconditioner(std::unique_ptr<PreconditionerBase> preconditioner) {
  preconditioner_ = std::move(preconditioner);
}
}  // namespace ax::math
