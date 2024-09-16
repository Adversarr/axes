#pragma once
#include "ax/core/buffer/buffer_view.hpp"

namespace ax::math {

class RealCompressedMatrixBase {
public:
  virtual ~RealCompressedMatrixBase() = default;

  // computes y = alpha * A * x + beta * y
  virtual void Multiply(ConstRealBufferView x, RealBufferView y, Real alpha, Real beta) const = 0;

  // computes y = alpha * A^T * x + beta * y
  virtual void TransposeMultiply(ConstRealBufferView x, RealBufferView y, Real alpha,
                                 Real beta) const
      = 0;

  // returns the A-inner product of x and y, there is a default implementation
  //   1.      y = A * x
  //   2. result = dot(y, x)
  virtual Real InnerProduct(ConstRealBufferView x, ConstRealBufferView y) const;

  // setup the mat descriptor
  virtual void Finish();

protected:
  BufferPtr<Index> ptrs_;     // row_ptrs if CSR, col_ptrs if CSC
  BufferPtr<Index> indices_;  // col_indices if CSR, row_indices if CSC
  BufferPtr<Real> values_;    // values
  size_t rows_{0};            // number of rows in block
  size_t cols_{0};            // number of columns in block
  size_t block_size_{1};      // block size
  BufferDevice device_;       // device
};

}  // namespace ax::math