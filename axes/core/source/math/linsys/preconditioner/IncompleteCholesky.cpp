#include "ax/math/linsys/preconditioner/IncompleteCholesky.hpp"

namespace ax::math {

void Preconditioner_IncompleteCholesky::AnalyzePattern() {
  impl_.analyzePattern(cached_problem_->A_);
  AX_THROW_IF_FALSE(impl_.info() == Eigen::Success, "IncompleteCholesky failed to compute.");
}

void Preconditioner_IncompleteCholesky::Factorize() {
  impl_.factorize(cached_problem_->A_);
  AX_THROW_IF_FALSE(impl_.info() == Eigen::Success, "IncompleteCholesky failed to compute.");
}

vecxr Preconditioner_IncompleteCholesky::Solve(vecxr const &b) { return impl_.solve(b); }

}  // namespace ax::math
