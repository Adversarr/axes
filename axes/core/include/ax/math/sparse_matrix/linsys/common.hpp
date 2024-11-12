#pragma once
#include "ax/core/buffer/buffer_view.hpp"
#include "ax/math/sparse_matrix/sparse_matrix.hpp"

namespace ax::math {

AX_DEFINE_ENUM_CLASS(GeneralSparseSolverKind,
                     ConjugateGradient,  // block based conjugate gradient.
                     Downcast,           // Downcast to the normal solver.
);

AX_DEFINE_ENUM_CLASS(GeneralPreconditionerKind,
                     IncompleteCholesky,  // Incomplete Cholesky preconditioner.
                     Jacobi,              // Jacobi preconditioner.
                     BlockJacobi,         // Block Jacobi preconditioner.
                     Identity,            // Identity preconditioner.
                     FSAI0,               // FSAI0 preconditioner.
                     ILU,                 // ILU preconditioner.
)

class GeneralSparsePreconditionerBase;
class GeneralSparseSolverBase;

using RealSparseMatrixPtr = std::shared_ptr<RealCompressedMatrixBase>;
using ConstRealSparseMatrixPtr = std::shared_ptr<const RealCompressedMatrixBase>;

struct BlockedLinsysSolveStatus {
  bool converged_{false};
  size_t iter_{0};
  Real l2_err_{0};
};

}  // namespace ax::math