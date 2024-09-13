#pragma once
#include "ax/core/buffer/buffer_view.hpp"
#include "ax/math/sparse.hpp"

namespace ax::math {

/**
 * @brief An optimized version of sparse matrix. support both Host and Device.
 *        It use compressed row storage (CRS) format.
 *
 */
class RealSparseMatrixCompressed {
public:
  RealSparseMatrixCompressed() = default;

  RealSparseMatrixCompressed(size_t rows, size_t cols, BufferDevice device)
      : rows_(rows), cols_(cols), device_(device) {}

  RealSparseMatrixCompressed(const RealSparseMatrixCompressed&) = default;
  RealSparseMatrixCompressed(RealSparseMatrixCompressed&& other) noexcept = default;

  explicit RealSparseMatrixCompressed(const RealSparseMatrix& mat, BufferDevice device);

  void /*NOLINT: google-default-argument*/ RightMultiplyTo(ConstRealBufferView x, RealBufferView y,
                                                           Real alpha = 1, Real beta = 0) const;

  // Set the data of the matrix.
  void SetData(BufferView<const int> row_ptrs, BufferView<const int> col_indices,
               BufferView<const Real> values);

  // Setup the internal data from the triplets.
  void SetFromTriplets(const RealSparseCOO& coo);

  // Prune the matrix by removing the elements with absolute value less than eps.
  void Prune(Real eps = math::epsilon<Real>);

  // Convert to Eigen SparseMatrix
  RealSparseMatrix ToSparseMatrix() const;

  // After construction of the matrix, you can call this function to enable the optimized
  // computation.
  void Finish() const;

private:
  BufferPtr<int> row_ptrs_;
  BufferPtr<int> col_indices_;
  BufferPtr<Real> values_;

  size_t rows_{0};
  size_t cols_{0};
  BufferDevice device_;
  mutable std::shared_ptr<void> mat_descr_;
};

}  // namespace ax::math