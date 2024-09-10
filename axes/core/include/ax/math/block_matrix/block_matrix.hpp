#pragma once
#include "ax/core/buffer/buffer.hpp"
#include "ax/math/sparse.hpp"
namespace ax::math {

/**
 * @brief Block sparse matrix.
 * @note  Usage: https://docs.nvidia.com/cuda/cusparse/index.html#block-sparse-row-bsr
 * 
 */
class RealBlockMatrix {
public:
  RealBlockMatrix() = default;
  RealBlockMatrix(const RealBlockMatrix&);

  RealBlockMatrix(size_t rows, size_t cols, size_t block_size,
                  BufferDevice device = BufferDevice::Host)
      : rows_(rows), cols_(cols), block_size_(block_size), device_(device) {}

  void SetData(BufferView<const int> block_row_ptrs,
               BufferView<const int> block_col_indices,
               BufferView<const Real> block_values);

  void Set(const RealBlockMatrix& other);

  void SetData(BufferPtr<int> block_row_ptrs,
               BufferPtr<int> block_col_indices,
               BufferPtr<Real> block_values);

  size_t NumNonZeroBlocks() const { return block_row_ptrs_->Size() - 1; }

  // 2D rhs is required.
  void RightMultiplyTo(BufferView<const Real> rhs, BufferView<Real> dst, Real alpha = 1,
                       Real beta = 0) const;
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
  BufferDevice GetDevice() const { return device_; }

  math::RealSparseMatrix ToSparseMatrix() const;

  // Return the internal buffers.
  BufferView<const int> BlockRowPtrsView() const;
  BufferView<const int> BlockColIndicesView() const;
  BufferView<const Real> BlockValuesView() const;
private:
  void EnsureMatDesc() const;

  // Compress in Block CSR.
  BufferPtr<int> block_row_ptrs_;     // 1D buffer. size == rows + 1
  BufferPtr<int> block_col_indices_;  // 1D buffer. size == nnz of block
  BufferPtr<Real> block_values_;  // a 3D buffer. (BlockSize.Row, BlockSize.Col, nnz of Blocks)

  // Shape of the matrix, count in block
  size_t rows_{0};
  size_t cols_{0};
  size_t block_size_{0};

  BufferDevice device_;
  mutable std::shared_ptr<void> mat_desc_;
};

// NOTE: We use the RealBufferView to represent a view of a block vector.
// the vector is:
//    =1 col, if the shape of view is [block_size, nrows].
//    >1 col, if the shape of view is [block_size, nrows, ncols].
// but for now, we do not support the second version.

}  // namespace ax::math