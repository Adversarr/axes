#pragma once
#include "ax/core/buffer/buffer_view.hpp"
#include "ax/math/sparse.hpp"
#include "ax/math/sparse_matrix/sparse_matrix.hpp"

namespace ax::math {

/**
 * @brief An optimized version of sparse matrix. support both Host and Device.
 *        It use compressed row storage (CRS) format.
 *
 */
class RealCSRMatrix : public RealCompressedMatrixBase {
public:
  RealCSRMatrix() = default;

  RealCSRMatrix(size_t rows, size_t cols, BufferDevice device);

  RealCSRMatrix(const RealCSRMatrix&) = default;
  RealCSRMatrix(RealCSRMatrix&& other) noexcept = default;

  explicit RealCSRMatrix(const RealSparseMatrix& mat, BufferDevice device);

  void Multiply(ConstRealBufferView x, RealBufferView y, Real alpha, Real beta) const override;

  // computes y = alpha * A^T * x + beta * y
  void TransposeMultiply(ConstRealBufferView x, RealBufferView y, Real alpha,
                         Real beta) const override;

  // Set the data of the matrix.
  void SetData(ConstIntBufferView row_ptrs, ConstIntBufferView col_indices,
               ConstRealBufferView values);

  // Setup the internal data from the triplets.
  void SetFromTriplets(const RealSparseCOO& coo);

  // Convert to Eigen SparseMatrix
  RealSparseMatrix ToSparseMatrix() const override;

  // After construction of the matrix, you can call this function to enable the optimized
  // computation.
  void Finish() override;

  std::unique_ptr<RealCSRMatrix> ToCSR() const final;
  std::unique_ptr<RealCompressedMatrixBase> Transfer(BufferDevice device) const override;

private:
  std::shared_ptr<void> mat_descr_;
};

}  // namespace ax::math