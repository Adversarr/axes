#include "axes/math/linsys/sparse/BiCGSTAB.hpp"
namespace ax::math {

Status SparseSolver_BiCGSTAB::Analyse(LinsysProblem_Sparse const &problem) {
  solver_.compute(problem.A_);
  if (solver_.info() != Eigen::Success) {
    return utils::FailedPreconditionError("SparseSolver_BiCGSTAB: factorization failed");
  }

  AX_RETURN_OK();
}

LinsysSolveResult SparseSolver_BiCGSTAB::Solve(vecxr const &b, vecxr const &x0) {
  if (!preconditioner_) {
    // TODO: Fix bug
    // solver_.setMaxIterations(options.GetDefault<idx>("max_iter", 100));
    vecxr x;
    if (x0.size() > 0) {
      x = solver_.solveWithGuess(b, x0);
    } else {
      x = solver_.solve(b);
    }
    LinsysSolveResultImpl impl(x, solver_.info() == Eigen::Success);
    impl.num_iter_ = solver_.iterations();
    impl.l2_err_ = solver_.error();
    return impl;
  } else {
    AX_CHECK(false) << "This branch have not been implemented yet";
  }
  AX_UNREACHABLE();
}

}  // namespace ax::math