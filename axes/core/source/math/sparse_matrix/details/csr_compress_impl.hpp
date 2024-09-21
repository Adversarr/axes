#pragma once
#include "ax/math/sparse_matrix/csr.hpp"

namespace ax::math::details {

std::shared_ptr<void> create_csr_compress_desc_gpu(IntBufferView row_ptrs,
                                                   IntBufferView col_indices,
                                                   BufferView<Real> values, size_t rows,
                                                   size_t cols);

void compute_csr_spmv_gpu(BufferView<const Real> x, BufferView<Real> y, Real alpha, Real beta,
                          std::shared_ptr<void> desc);

void compute_csr_spmv_cpu(ConstRealBufferView x, RealBufferView y, Real alpha, Real beta,
                          BufferView<const int> row_ptrs, BufferView<const int> col_indices,
                          BufferView<const Real> values, size_t cols);

void compute_csr_spmv_transpose_gpu(BufferView<const Real> x, BufferView<Real> y, Real alpha,
                                    Real beta, std::shared_ptr<void> desc);

void compute_csr_spmv_transpose_cpu(ConstRealBufferView x, RealBufferView y, Real alpha, Real beta,
                                    BufferView<const int> row_ptrs,
                                    BufferView<const int> col_indices,
                                    BufferView<const Real> values, size_t cols);

}  // namespace ax::math::details
