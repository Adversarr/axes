#include "./csr_compress_impl.hpp"
#include "ax/core/buffer/eigen_support.hpp"
#include "ax/core/buffer/for_each.hpp"
#include "ax/math/utils/formatting.hpp"

namespace ax::math::details {

void compute_csr_spmv_cpu(ConstRealBufferView x, RealBufferView y, Real alpha, Real beta,
                          BufferView<const int> row_ptrs, BufferView<const int> col_indices,
                          BufferView<const Real> values, size_t cols) {
  const size_t rows = row_ptrs.Shape().X() - 1;
  const size_t nnz = values.Shape().Z();

  // auto job = [&row_ptrs, &col_indices, &values, &x, alpha, beta, &y](size_t row, size_t colj) {
  //   const int row_start = row_ptrs(row);
  //   const int row_end = row_ptrs(row + 1);
  //   Real sum = 0;
  //   for (int i = row_start; i < row_end; ++i) {
  //     size_t col = static_cast<size_t>(col_indices(static_cast<size_t>(i)));
  //     sum += values(static_cast<size_t>(i)) * x(col, colj);
  //   }
  //
  //   // alpha Ax + beta y
  //   y(row, colj) = alpha * sum + beta * y(row, colj);
  // };
  //
  // size_t cols = x.Shape().Y() == 0 ? 1 : x.Shape().Y();
  // if (nnz > 1 << 16) {
  //   par_for_each_indexed(Dim(rows, cols), job);
  // } else {
  //   for_each_indexed(Dim(rows, cols), job);
  // }

  Eigen::Map<const Eigen::SparseMatrix<Real, Eigen::RowMajor, int>> mat(
      static_cast<Index>(rows), static_cast<Index>(cols), static_cast<Index>(nnz), row_ptrs.Data(),
      col_indices.Data(), values.Data());

  auto xmap = view_as_matrix_full<const RealMatrixX>(x);
  auto ymap = view_as_matrix_full<RealMatrixX>(y);
  ymap *= beta;
  ymap.noalias() += alpha * mat * xmap;
}

void compute_csr_spmv_transpose_cpu(ConstRealBufferView x, RealBufferView y, Real alpha, Real beta,
                                    BufferView<const int> row_ptrs,
                                    BufferView<const int> col_indices,
                                    BufferView<const Real> values, size_t cols) {
  const size_t rows = row_ptrs.Shape().X() - 1;
  const size_t nnz = values.Shape().Z();
  // Not parallelizable.

  Eigen::Map<const Eigen::SparseMatrix<Real, Eigen::RowMajor, int>> mat(
      static_cast<Index>(rows), static_cast<Index>(cols), static_cast<Index>(nnz), row_ptrs.Data(),
      col_indices.Data(), values.Data());

  auto xmap = view_as_matrix_full<const RealMatrixX>(x);
  auto ymap = view_as_matrix_full<RealMatrixX>(y);
  ymap *= beta;
  ymap.noalias() += alpha * mat.transpose() * xmap;
}

}  // namespace ax::math::details