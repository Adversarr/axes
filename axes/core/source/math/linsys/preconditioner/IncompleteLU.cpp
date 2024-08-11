#include "ax/math/linsys/preconditioner/IncompleteLU.hpp"

namespace ax::math {

void Preconditioner_IncompleteLU::AnalyzePattern() {
  impl_.analyzePattern(cached_problem_->A_);
  AX_THROW_IF_FALSE(impl_.info() == Eigen::Success, "The factorization has not been computed.");
}

void Preconditioner_IncompleteLU::Factorize() {
  impl_.factorize(cached_problem_->A_);
  AX_THROW_IF_FALSE(impl_.info() == Eigen::Success, "The factorization has not been computed.");
}

matxxr Preconditioner_IncompleteLU::Solve(matxxr const &b) { return impl_.solve(b); }
}  // namespace ax::math
