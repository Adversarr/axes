#pragma once
#include "ax/core/buffer/buffer.hpp"
#include "ax/math/common.hpp"

namespace ax::math {

class BlockMatrix {
public:
  BlockMatrix() = default;
  BlockMatrix(const BlockMatrix&) = default;

  BlockMatrix(size_t rows, size_t cols) : rows_(rows), cols_(cols) {}

  void SetData(size_t rows, size_t cols, const size_t* block_row_ptrs, const size_t* block_col_indices,
               const Real* block_values, size_t nnz, BufferDevice from_device);

  // 2D rhs is required.
  void RightMultiplyTo(BufferView<const Real> rhs, BufferView<Real> dst) const;
  void LeftMultiplyTo(BufferView<const Real> rhs, BufferView<Real> dst) const;

  size_t BlockedRows() const { return rows_; }

  size_t BlockedCols() const { return cols_; }

  size_t Rows() const { return rows_ * block_values_->Shape().X(); }

  size_t Cols() const { return cols_ * block_values_->Shape().Y(); }

  BufferDevice GetDevice() const { return block_values_->Device(); }

  math::RealSparseMatrix ToSparseMatrix() const;

private:
  // Compress in Block CSR.
  BufferPtr<size_t> block_row_ptrs_;     // 1D buffer. size == rows + 1
  BufferPtr<size_t> block_col_indices_;  // 1D buffer. size == nnz of block
  BufferPtr<Real> block_values_;  // a 3D buffer. (BlockSize.Row, BlockSize.Col, nnz of Blocks)

  // Shape of the matrix, count in block
  size_t rows_{0};
  size_t cols_{0};
};

}  // namespace ax::math