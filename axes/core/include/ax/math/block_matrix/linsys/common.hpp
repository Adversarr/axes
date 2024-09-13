#pragma once
#include "ax/core/buffer/buffer_view.hpp"
#include "ax/math/block_matrix/block_matrix.hpp"

namespace ax::math {

AX_DEFINE_ENUM_CLASS(BlockSolverKind,
                     ConjugateGradient,  // block based conjugate gradient.
                     Downcast,           // Downcast to the normal solver.
);

AX_DEFINE_ENUM_CLASS(BlockPreconditionerKind,
                     Jacobi,       // Jacobi preconditioner.
                     BlockJacobi,  // Block Jacobi preconditioner.
                     Identity,     // Identity preconditioner.
)

class BlockPreconditionerBase;
class BlockSolverBase;

/**
 * @brief Describes the blocked linear system problem.
 * 
 */
struct BlockedLinsysProblem {
  // Problem Description
  RealBlockMatrix A_;

  // For Iterative Solvers: Solution Requirement, may be ignored.
  Real l2_tol_{1e-6};
  Real linf_tol_{1e-6};

  // Additional checkers.
  std::function<bool(RealBufferView const&, RealBufferView const&)> converge_residual_;
  std::function<bool(RealBufferView const&)> converge_solution_;

  // we can just construct from a r-value.
  explicit BlockedLinsysProblem(RealBlockMatrix A) : A_(std::move(A)) {}
};

struct BlockedLinsysSolveStatus {
  bool converged_{false};
  size_t iter_{0};
  Real l2_err_{0};

  static BlockedLinsysSolveStatus Converged(size_t iter, Real l2_err) {
    return {true, iter, l2_err};
  }

  static BlockedLinsysSolveStatus NotConverged(size_t iter, Real l2_err) {
    return {false, iter, l2_err};
  }
};

}  // namespace ax::math