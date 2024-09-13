#include "./csr_compress_impl.hpp"
#include "ax/core/buffer/for_each.hpp"

namespace ax::math::details {

void compute_csr_spmv_cpu(ConstRealBufferView x, RealBufferView y, Real alpha, Real beta,
                          BufferView<const int> row_ptrs, BufferView<const int> col_indices,
                          BufferView<const Real> values) {
  const size_t rows = row_ptrs.Shape().X() - 1;
  const size_t nnz = values.Shape().Z();

  auto job = [&row_ptrs, &col_indices, &values, &x, alpha, beta, &y](size_t row, size_t colj) {
    const int row_start = row_ptrs(row);
    const int row_end = row_ptrs(row + 1);
    Real sum = 0;
    for (int i = row_start; i < row_end; ++i) {
      size_t col = static_cast<size_t>(col_indices(static_cast<size_t>(i)));
      sum += values(static_cast<size_t>(i)) * x(col, colj);
    }

    // alpha Ax + beta y
    y(row, colj) = alpha * sum + beta * y(row, colj);
  };

  size_t cols = x.Shape().Y() == 0 ? 1 : x.Shape().Y();
  if (rows > 4096) {
    par_for_each_indexed(Dim(rows, cols), job);
  } else {
    for_each_indexed(Dim(rows, cols), job);
  }
}

}  // namespace ax::math::details