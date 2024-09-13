#include "ax/math/block_matrix/linsys/solver.hpp"

#include "ax/math/block_matrix/linsys/preconditioner.hpp"

namespace ax::math {

void BlockSolverBase::SetProblem(RealBlockMatrix A) {
  auto problem = std::make_unique<BlockedLinsysProblem>(std::move(A));
  SetProblem(std::move(problem));
}

void BlockSolverBase::SetProblem(std::unique_ptr<BlockedLinsysProblem> problem) {
  problem_ = std::move(problem);
}

void BlockSolverBase::AnalyzePattern() {
  if (preconditioner_) {
    preconditioner_->SetProblem(problem_);
    preconditioner_->AnalyzePattern();
  }
}

void BlockSolverBase::Factorize() {
  if (preconditioner_) {
    preconditioner_->SetProblem(problem_);
    preconditioner_->Factorize();
  }
}

void BlockSolverBase::Compute() {
  AnalyzePattern();
  Factorize();
}
}  // namespace ax::math