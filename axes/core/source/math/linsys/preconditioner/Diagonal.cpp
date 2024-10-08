#include "ax/math/linsys/preconditioner/Diagonal.hpp"

namespace ax::math {

void Preconditioner_Diagonal::AnalyzePattern() {
  impl_.analyzePattern(cached_problem_->A_);
  AX_THROW_IF_FALSE(impl_.info() == Eigen::Success, "The factorization has not been computed.");
}

void Preconditioner_Diagonal::Factorize() {
  impl_.factorize(cached_problem_->A_);
  AX_THROW_IF_FALSE(impl_.info() == Eigen::Success, "The factorization has not been computed.");
}

RealMatrixX Preconditioner_Diagonal::Solve(RealMatrixX const &b) { return impl_.solve(b); }

}  // namespace ax::math
