#pragma once

#include "ax/core/buffer/buffer_view.hpp"
#include "ax/math/sparse.hpp"

namespace ax::math {

// The most important.
class RealCSRMatrix;

class RealCompressedMatrixBase {
public:
  RealCompressedMatrixBase() = default;
  RealCompressedMatrixBase(size_t rows, size_t cols, BufferDevice device)
      : rows_(rows), cols_(cols), device_(device) {}

  RealCompressedMatrixBase(size_t rows, size_t cols, size_t block_size, BufferDevice device)
      : rows_(rows), cols_(cols), block_size_(block_size), device_(device) {}

  virtual ~RealCompressedMatrixBase() = default;

  // computes y = alpha * A * x + beta * y
  virtual void Multiply(ConstRealBufferView x, RealBufferView y, Real alpha, Real beta) const = 0;

  // computes y = alpha * A^T * x + beta * y
  virtual void TransposeMultiply(ConstRealBufferView x, RealBufferView y, Real alpha,
                                 Real beta) const;

  // returns the A-inner product of x and y, there is a default implementation
  //   1.      y = A * x
  //   2. result = dot(y, x)
  virtual Real InnerProduct(ConstRealBufferView x, ConstRealBufferView y) const;

  // setup the mat descriptor
  virtual void Finish();

  size_t BlockSize() const { return block_size_; }

  BufferDevice Device() const { return device_; }

  const BufferPtr<int>& RowPtrs() const { return row_ptrs_; }

  const BufferPtr<int>& ColIndices() const { return col_indices_; }

  const BufferPtr<Real>& Values() const { return values_; }

  size_t Rows() const { return block_size_ * rows_; }

  size_t BlockedRows() const { return rows_; }

  size_t Cols() const { return block_size_ * cols_; }

  size_t BlockedCols() const { return cols_; }

  void MarkAsSymmetric(bool is_symm = true) { is_symm_ = is_symm; }

  virtual std::unique_ptr<RealCSRMatrix> ToCSR() const = 0;

  virtual math::RealSparseMatrix ToSparseMatrix() const = 0;

  virtual std::unique_ptr<RealCompressedMatrixBase> Transfer(BufferDevice device) const = 0;

protected:
  BufferPtr<int> row_ptrs_;     // row_ptrs if CSR, col_ptrs if CSC
  BufferPtr<int> col_indices_;  // col_indices if CSR, row_indices if CSC
  BufferPtr<Real> values_;      // values
  size_t rows_{0};              // number of rows in block
  size_t cols_{0};              // number of columns in block
  size_t block_size_{1};        // block size
  BufferDevice device_;         // device
  bool is_symm_{false};         // is symmetric, if True, may perform TransposeMultiply faster
};

}  // namespace ax::math