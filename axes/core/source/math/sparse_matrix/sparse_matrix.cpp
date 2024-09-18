#include "ax/math/sparse_matrix/sparse_matrix.hpp"

#include "ax/core/excepts.hpp"

namespace ax::math {

Real RealCompressedMatrixBase::InnerProduct(ConstRealBufferView x, ConstRealBufferView y) const {
  AX_THROW_RUNTIME_ERROR("Not implemented yet.");
  AX_UNREACHABLE();
}

void RealCompressedMatrixBase::TransposeMultiply(ConstRealBufferView x, RealBufferView y, Real alpha,
                                                 Real beta) const {
  if (is_symm_) {
    Multiply(x, y, alpha, beta);
    return;
  }

  AX_THROW_RUNTIME_ERROR("Not implemented yet.");
  AX_UNREACHABLE();
}

void RealCompressedMatrixBase::Finish() {
  // Do nothing.
}

}  // namespace ax::math