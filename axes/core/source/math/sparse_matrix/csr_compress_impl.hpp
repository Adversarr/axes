#pragma once
#include "ax/math/sparse_matrix/csr_compress.hpp"

namespace ax::math::details {

std::shared_ptr<void> create_csr_compress_desc_gpu(BufferView<int> row_ptrs,
                                                   BufferView<int> col_indices,
                                                   BufferView<Real> values, size_t rows,
                                                   size_t cols);

void compute_csr_spmv_gpu(BufferView<const Real> x, BufferView<Real> y, Real alpha, Real beta,
                          std::shared_ptr<void> desc);

void compute_csr_spmv_cpu(ConstRealBufferView x, RealBufferView y, Real alpha, Real beta,
                          BufferView<const int> row_ptrs, BufferView<const int> col_indices,
                          BufferView<const Real> values);

}  // namespace ax::math::details
