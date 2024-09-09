#include "ax/core/buffer/for_each.hpp"
#include "ax/utils/ndrange.hpp"
#include "block_matrix_matmul_impl.hpp"
namespace ax::math::details {

void do_mv(
  const Real* in_block_value,
  const Real* in_rhs,
  Real* out_dst,
  size_t block_size
) {
  // rhs[j] = sum_k in_block_value[i, k] * in_rhs[k, j], we are using col-major
  // linear_index = i + j * block_size
#pragma omp simd
  for (size_t i = 0; i < block_size; ++i) {
    for (size_t j = 0; j < block_size; ++j) {
      out_dst[i] += in_block_value[i + j * block_size] * in_rhs[j];
    }
  }
}

void block_matrix_matmul_cpu(
  size_t rows,
  size_t /* cols */,
  BufferView<const Real> block_values,
  BufferView<const size_t> block_row_ptrs,
  BufferView<const size_t> block_col_indices,
  BufferView<const Real> rhs,
  BufferView<Real> dst) {
  // For each row block, do parallel.
  // Alg: out[i] = sum_j block_values[i, j] * rhs[j]
  for_each_indexed(Dim{rows}, [&](size_t row) {
    size_t block_curr = block_row_ptrs(row);
    size_t block_next = block_row_ptrs(row + 1);
    size_t block_size = block_values.Shape().X();
    Real* out_dst = dst.Offset(0, row);

    for (size_t local = 0; local < block_size; ++local) {
      out_dst[local] = 0;
    }

    for (size_t block_id: utils::range<size_t>(block_curr, block_next)) {
      size_t col = block_col_indices(block_id);
      const Real* in_block = block_values.Offset(0, 0, block_id);
      const Real* in_rhs = rhs.Offset(0, col);
      do_mv(in_block, in_rhs, out_dst, block_size);
    }
  });
}

}