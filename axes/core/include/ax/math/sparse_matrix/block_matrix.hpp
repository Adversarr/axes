#pragma once
#include "ax/core/buffer/buffer.hpp"
#include "ax/math/sparse.hpp"
#include "ax/math/sparse_matrix/sparse_matrix.hpp"

namespace ax::math {

/**
 * @brief Block sparse matrix.
 * @note  Usage: https://docs.nvidia.com/cuda/cusparse/index.html#block-sparse-row-bsr
 *
 */
class RealBlockMatrix : public RealCompressedMatrixBase {
public:
  RealBlockMatrix() = default;
  RealBlockMatrix(const RealBlockMatrix&) = delete;
  RealBlockMatrix& operator=(const RealBlockMatrix&) = delete;
  RealBlockMatrix(RealBlockMatrix&&) = default;
  RealBlockMatrix& operator=(RealBlockMatrix&&) = default;

  RealBlockMatrix(size_t rows, size_t cols, size_t block_size,
                  BufferDevice device = BufferDevice::Host)
      : RealCompressedMatrixBase(rows, cols, block_size, device) {}

  void SetData(BufferView<const int> block_row_ptrs, BufferView<const int> block_col_indices,
               BufferView<const Real> block_values);

  void Set(const RealBlockMatrix& other);

  void SetData(BufferPtr<int> block_row_ptrs, BufferPtr<int> block_col_indices,
               BufferPtr<Real> block_values);

  void SetFromBlockedTriplets(ConstIntBufferView row, ConstIntBufferView col,
                              ConstRealBufferView values);

  // 2D rhs is required.
  void Multiply(BufferView<const Real> rhs, BufferView<Real> dst, Real alpha,
                Real beta) const override;
  void TransposeMultiply(ConstRealBufferView x, RealBufferView y, Real alpha,
                         Real beta) const override;

  math::RealSparseMatrix ToSparseMatrix() const override;

  void Finish() override;

  std::unique_ptr<RealCSRMatrix> ToCSR() const override;
  std::unique_ptr<RealCompressedMatrixBase> Transfer(BufferDevice device) const override;
};

// NOTE: We use the RealBufferView to represent a view of a block vector.
// the vector is:
//    =1 col, if the shape of view is [block_size, nrows].
//    >1 col, if the shape of view is [block_size, nrows, ncols].
// but for now, we do not support the second version.

}  // namespace ax::math