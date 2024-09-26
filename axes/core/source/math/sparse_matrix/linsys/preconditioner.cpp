#include "ax/math/sparse_matrix/linsys/preconditioner.hpp"

#include "ax/math/sparse_matrix/linsys/preconditioner/block_jacobi.hpp"
#include "ax/math/sparse_matrix/linsys/preconditioner/fsai0.hpp"
#include "ax/math/sparse_matrix/linsys/preconditioner/ic.hpp"
#include "ax/math/sparse_matrix/linsys/preconditioner/jacobi.hpp"

namespace ax::math {

void GeneralSparsePreconditionerBase::SetProblem(ConstRealSparseMatrixPtr mat) {
  mat_ = mat;
}

std::unique_ptr<GeneralSparsePreconditionerBase> GeneralSparsePreconditionerBase::Create(
    GeneralPreconditionerKind kind) {
  switch (kind) {
    case GeneralPreconditionerKind::Identity:
      AX_NOT_IMPLEMENTED();

    case GeneralPreconditionerKind::Jacobi:
      return std::make_unique<GeneralSparsePreconditioner_Jacobi>();

    case GeneralPreconditionerKind::BlockJacobi:
      return std::make_unique<GeneralSparsePreconditioner_BlockJacobi>();

    case GeneralPreconditionerKind::FSAI0:
      return std::make_unique<GeneralSparsePreconditioner_FSAI0>();

    case GeneralPreconditionerKind::IncompleteCholesky:
      return std::make_unique<GeneralSparsePreconditioner_IncompleteCholesky>();

    default:
      AX_THROW_INVALID_ARGUMENT("Unknown kind");
  }

  AX_UNREACHABLE();
}

}  // namespace ax::math