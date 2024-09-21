#include "ax/core/buffer/for_each.hpp"
#include "ax/math/buffer_blas.hpp"
#include "ax/math/sparse_matrix/details/matmul_impl.hpp"
#include "ax/utils/ndrange.hpp"

namespace ax::math::details {

static AX_FORCE_INLINE void do_mv(const Real* in_block_value, const Real* in_rhs, Real* out_dst,
                                  size_t block_size, Real alpha) {
  // rhs[j] = sum_k in_block_value[i, k] * in_rhs[k, j], we are using col-major
  // linear_index = i + j * block_size
  for (size_t j = 0; j < block_size; ++j) {
    for (size_t i = 0; i < block_size; ++i) {
      out_dst[i] += alpha * (in_block_value[i + j * block_size] * in_rhs[j]);
    }
  }
}

static AX_FORCE_INLINE void do_mv_transpose(const Real* in_block_value, const Real* in_rhs,
                                            Real* out_dst, size_t block_size, Real alpha) {
  // rhs[j] = sum_k in_block_value[i, k] * in_rhs[j, k], we are using col-major
  // linear_index = i + j * block_size
  for (size_t j = 0; j < block_size; ++j) {
    for (size_t i = 0; i < block_size; ++i) {
      out_dst[j] += alpha * (in_block_value[i + j * block_size] * in_rhs[i]);
    }
  }
}

void block_matrix_mv_cpu(size_t rows, size_t /* cols */, BufferView<const Real> block_values,
                             BufferView<const int> block_row_ptrs,
                             BufferView<const int> block_col_indices, BufferView<const Real> rhs,
                             BufferView<Real> dst, Real alpha, Real beta,
                             std::shared_ptr<void> /* descr_type_erased */) {
  // For each row block, do parallel.
  // Alg: out[i] = sum_j block_values[i, j] * rhs[j]
  size_t nnz = block_col_indices.Shape().X();
  auto job = [=](size_t row) mutable {
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
  if (nnz > 1 << 15) {
    par_for_each_indexed(Dim{rows}, job);
  } else {
    for_each_indexed(Dim{rows}, job);
  }
}

void block_matrix_transpose_mv_cpu(size_t rows, size_t /* cols */,
                                   BufferView<const Real> block_values,
                                   BufferView<const int> block_row_ptrs,
                                   BufferView<const int> block_col_indices,
                                   BufferView<const Real> rhs, BufferView<Real> dst, Real alpha,
                                   Real beta, std::shared_ptr<void> /* descr_type_erased */) {
  // sequential.
  math::buffer_blas::scal(beta, dst);  // dst <- beta * dst.

  for (size_t row = 0; row < rows; ++row) {
    size_t block_curr = static_cast<size_t>(block_row_ptrs(row));
    size_t block_next = static_cast<size_t>(block_row_ptrs(row + 1));
    size_t block_size = block_values.Shape().X();
    const Real* in_rhs = rhs.Offset(0, row);  // rhs[row]

    for (size_t block_id = block_curr; block_id < block_next; ++block_id) {
      size_t col = static_cast<size_t>(block_col_indices(block_id));
      Real* out_dst = dst.Offset(0, col);                          // dst[col]
      const Real* in_block = block_values.Offset(0, 0, block_id);  // mat[row, col]
      do_mv_transpose(in_block, in_rhs, out_dst, block_size, alpha);
    }
  }
}

}  // namespace ax::math::details