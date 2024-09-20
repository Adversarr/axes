#include "ax/core/buffer/for_each.hpp"
#include "ax/math/sparse_matrix/details/matmul_impl.hpp"
#include "ax/utils/ndrange.hpp"

namespace ax::math::details {

static AX_FORCE_INLINE void do_mv(const Real* in_block_value, const Real* in_rhs, Real* out_dst,
                                  size_t block_size, Real alpha) {
  // rhs[j] = sum_k in_block_value[i, k] * in_rhs[k, j], we are using col-major
  // linear_index = i + j * block_size
  for (size_t j = 0; j < block_size; ++j) {
#pragma omp simd
    for (size_t i = 0; i < block_size; ++i) {
      out_dst[i] += alpha * (in_block_value[i + j * block_size] * in_rhs[j]);
    }
  }
}

void block_matrix_matmul_cpu(size_t rows, size_t /* cols */, BufferView<const Real> block_values,
                             BufferView<const int> block_row_ptrs,
                             BufferView<const int> block_col_indices, BufferView<const Real> rhs,
                             BufferView<Real> dst, Real alpha, Real beta,
                             std::shared_ptr<void> /* descr_type_erased */) {
  // For each row block, do parallel.
  // Alg: out[i] = sum_j block_values[i, j] * rhs[j]
  auto job = [&](size_t row) {
    size_t block_curr = static_cast<size_t>(block_row_ptrs(row));
    size_t block_next = static_cast<size_t>(block_row_ptrs(row + 1));
    size_t block_size = block_values.Shape().X();
    Real* out_dst = dst.Offset(0, row);

#pragma unroll
    for (size_t local = 0; local < block_size; ++local) {
      out_dst[local] = beta * out_dst[local];
    }

    for (size_t block_id : utils::range<size_t>(block_curr, block_next)) {
      size_t col = static_cast<size_t>(block_col_indices(block_id));
      const Real* in_block = block_values.Offset(0, 0, block_id);
      const Real* in_rhs = rhs.Offset(0, col);
      do_mv(in_block, in_rhs, out_dst, block_size, alpha);
    }
  };
  if (rows > 4096) {
    par_for_each_indexed(Dim{rows}, job);
  } else {
    for_each_indexed(Dim{rows}, job);
  }
}

}  // namespace ax::math::details