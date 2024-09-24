#include "ax/math/sparse_matrix/linsys/preconditioner.hpp"

namespace ax::math {

void GeneralSparsePreconditionerBase::SetProblem(ConstRealSparseMatrixPtr mat) {
  mat_ = mat;
}

}  // namespace ax::math