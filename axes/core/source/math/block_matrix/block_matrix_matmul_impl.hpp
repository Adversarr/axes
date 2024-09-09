#pragma once
#include "ax/core/buffer/buffer_view.hpp"
namespace ax::math::details {

void block_matrix_matmul_cpu(
  size_t rows,
  size_t cols,
  BufferView<const Real> block_values,
  BufferView<const size_t> block_row_ptrs,
  BufferView<const size_t> block_col_indices,
  BufferView<const Real> rhs,
  BufferView<Real> dst);


void block_matrix_matmul_gpu(
  size_t rows,
  size_t cols,
  BufferView<const Real> block_values,
  BufferView<const size_t> block_row_ptrs,
  BufferView<const size_t> block_col_indices,
  BufferView<const Real> rhs,
  BufferView<Real> dst);

}