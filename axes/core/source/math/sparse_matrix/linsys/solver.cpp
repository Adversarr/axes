#include "ax/math/sparse_matrix/linsys/solver.hpp"

#include "ax/math/sparse_matrix/linsys/preconditioner.hpp"

namespace ax::math {

void GeneralSparseSolverBase::SetProblem(RealSparseMatrixPtr mat) {
  mat_ = std::move(mat);
}

void GeneralSparseSolverBase::AnalyzePattern() {
  if (preconditioner_) {
    preconditioner_->SetProblem(mat_);
    preconditioner_->AnalyzePattern();
  }
}

void GeneralSparseSolverBase::Factorize() {
  if (preconditioner_) {
    preconditioner_->SetProblem(mat_);
    preconditioner_->Factorize();
  }
}

void GeneralSparseSolverBase::Compute() {
  AnalyzePattern();
  Factorize();
}

}  // namespace ax::math