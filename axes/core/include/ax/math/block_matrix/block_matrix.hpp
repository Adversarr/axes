#pragma once
#include "ax/core/buffer/buffer.hpp"
#include "ax/math/common.hpp"

namespace ax::math {

/**
 * @brief Block sparse matrix.
 * @note  Usage: https://docs.nvidia.com/cuda/cusparse/index.html#block-sparse-row-bsr
 * 
 */
class RealBlockMatrix {
public:
  RealBlockMatrix() = default;
  RealBlockMatrix(const RealBlockMatrix&) = default;

  RealBlockMatrix(size_t rows, size_t cols, size_t block_size)
      : rows_(rows), cols_(cols), block_size_(block_size) {}

  void SetData(BufferView<const size_t> block_row_ptrs,
               BufferView<const size_t> block_col_indices,
               BufferView<const Real> block_values);

  size_t NumNonZeroBlocks() const { return block_row_ptrs_->Size() - 1; }

  // 2D rhs is required.
  void RightMultiplyTo(BufferView<const Real> rhs, BufferView<Real> dst) const;
  void LeftMultiplyTo(BufferView<const Real> rhs, BufferView<Real> dst) const;

  // The number of row blocks in the matrix.
  size_t BlockedRows() const { return rows_; }
  // The number of column blocks in the matrix.
  size_t BlockedCols() const { return cols_; }
  // Size of block
  size_t BlockSize() const { return block_size_; }

  // The number of rows in the matrix.
  size_t Rows() const { return rows_ * block_size_; }
  // The number of columns in the matrix.
  size_t Cols() const { return cols_ * block_size_; }

  // Return the device of the internal buffers.
  BufferDevice GetDevice() const { return block_values_->Device(); }

  math::RealSparseMatrix ToSparseMatrix() const;

  // Return the internal buffers.
  BufferView<const size_t> BlockRowPtrsView() const;
  BufferView<const size_t> BlockColIndicesView() const;
  BufferView<const Real> BlockValuesView() const;

private:
  // Compress in Block CSR.
  BufferPtr<size_t> block_row_ptrs_;     // 1D buffer. size == rows + 1
  BufferPtr<size_t> block_col_indices_;  // 1D buffer. size == nnz of block
  BufferPtr<Real> block_values_;  // a 3D buffer. (BlockSize.Row, BlockSize.Col, nnz of Blocks)

  // Shape of the matrix, count in block
  size_t rows_{0};
  size_t cols_{0};
  size_t block_size_{0};
};

}  // namespace ax::math