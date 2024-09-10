#pragma once
#include "ax/core/buffer/buffer_view.hpp"
namespace ax::math::details {

void block_matrix_matmul_cpu(
  size_t rows,
  size_t cols,
  BufferView<const Real> block_values,
  BufferView<const int> block_row_ptrs,
  BufferView<const int> block_col_indices,
  BufferView<const Real> rhs,
  BufferView<Real> dst, Real alpha, Real beta, void* descr_type_erased);


void block_matrix_matmul_gpu(
  size_t rows,
  size_t cols,
  BufferView<const Real> block_values,
  BufferView<const int> block_row_ptrs,
  BufferView<const int> block_col_indices,
  BufferView<const Real> rhs,
  BufferView<Real> dst, Real alpha, Real beta, void* descr_type_erased);

std::shared_ptr<void> create_mat_desc_default();

}