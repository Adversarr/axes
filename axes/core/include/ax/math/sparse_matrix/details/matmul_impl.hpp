#pragma once
#include "ax/core/buffer/buffer_view.hpp"

namespace ax::math::details {

void block_matrix_mv_cpu(size_t rows, size_t cols, BufferView<const Real> block_values,
                             BufferView<const int> block_row_ptrs,
                             BufferView<const int> block_col_indices, BufferView<const Real> rhs,
                             BufferView<Real> dst, Real alpha, Real beta,
                             std::shared_ptr<void> descr_type_erased);

void block_matrix_mv_gpu(size_t rows, size_t cols, BufferView<const Real> block_values,
                             BufferView<const int> block_row_ptrs,
                             BufferView<const int> block_col_indices, BufferView<const Real> rhs,
                             BufferView<Real> dst, Real alpha, Real beta,
                             std::shared_ptr<void> descr_type_erased);

void block_matrix_transpose_mv_cpu(size_t rows, size_t cols, BufferView<const Real> block_values,
                                   BufferView<const int> block_row_ptrs,
                                   BufferView<const int> block_col_indices, BufferView<const Real> rhs,
                                   BufferView<Real> dst, Real alpha, Real beta,
                                   std::shared_ptr<void> descr_type_erased);

void block_matrix_transpose_mv_gpu(size_t rows, size_t cols, BufferView<const Real> block_values,
                                   BufferView<const int> block_row_ptrs,
                                   BufferView<const int> block_col_indices, BufferView<const Real> rhs,
                                   BufferView<Real> dst, Real alpha, Real beta,
                                   std::shared_ptr<void> descr_type_erased);

std::shared_ptr<void> create_bsr_mat_desc(BufferView<int> row_ptrs, BufferView<int> col_indices,
                                          BufferView<Real> values, size_t rows, size_t cols);
}  // namespace ax::math::details