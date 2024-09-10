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
};

}  // namespace ax::math