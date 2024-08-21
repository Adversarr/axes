#include "ax/math/linsys/sparse/LeastSquaresConjugateGradient.hpp"
#include "ax/core/logging.hpp"

namespace ax::math {

void SparseSolver_LeastSquaresConjugateGradient::AnalyzePattern() {
  solver_.analyzePattern(cached_problem_->A_);
  AX_THROW_IF_FALSE(solver_.info() == Eigen::Success,
                    "SparseSolver_LeastSquaresConjugateGradient: factorization failed");
}

void SparseSolver_LeastSquaresConjugateGradient::Factorize() {
  solver_.factorize(cached_problem_->A_);
  AX_THROW_IF_FALSE(solver_.info() == Eigen::Success,
                    "SparseSolver_LeastSquaresConjugateGradient: factorization failed");
}

LinsysSolveResult SparseSolver_LeastSquaresConjugateGradient::Solve(RealMatrixX const &b,
                                                                    RealMatrixX const &x0) {
  if (!preconditioner_) {
    // TODO: Fix bug
    // solver_.setMaxIterations(options.GetDefault<Index>("max_iter", 100));
    RealVectorX x;
    if (x0.size() > 0) {
      x = solver_.solveWithGuess(b, x0);
    } else {
      x = solver_.solve(b);
    }
    LinsysSolveResult impl(x, solver_.info() == Eigen::Success);
    impl.num_iter_ = solver_.iterations();
    impl.l2_err_ = solver_.error();
    return impl;
  } else {
    AX_CHECK(false, "This branch have not been implemented yet");
  }

  AX_UNREACHABLE();
}

}  // namespace ax::math
