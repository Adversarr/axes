#include "ax/core/excepts.hpp"
#include "ax/math/details/cusparse_context.cuh"
#include "ax/math/sparse_matrix/details/matmul_impl.hpp"

#include "csr_compress_impl.hpp"

#include <cusparse.h>

namespace ax::math::details {

void block_matrix_mv_gpu(size_t rows, size_t cols,
                         BufferView<const Real> block_values,
                         BufferView<const int> block_row_ptrs,
                         BufferView<const int> block_col_indices,
                         BufferView<const Real> rhs, BufferView<Real> dst,
                         Real alpha, Real beta,
                         std::shared_ptr<void> descr_type_erased) {
  cusparseContext *handle = get_cusparse_handle();

  size_t nnzb = block_values.Shape().Z();
  size_t block_size = block_values.Shape().X();
  if (block_size > 1) {
    cusparseMatDescr_t descr =
        static_cast<cusparseMatDescr_t>(descr_type_erased.get());
    auto status = cusparseDbsrmv(
        handle, CUSPARSE_DIRECTION_COLUMN, CUSPARSE_OPERATION_NON_TRANSPOSE,
        rows, cols, nnzb, &alpha, descr, block_values.Data(),
        block_row_ptrs.Data(), block_col_indices.Data(), block_size, rhs.Data(),
        &beta, dst.Data());

    if (status != CUSPARSE_STATUS_SUCCESS) {
      AX_THROW_RUNTIME_ERROR("cusparseDbsrmv failed {}: {}",
                             cusparseGetErrorName(status),
                             cusparseGetErrorString(status));
    }
  } else {
    // CSR matrix
    compute_csr_spmv_gpu(flatten(rhs), flatten(dst), alpha, beta,
                         descr_type_erased);
  }
}

void block_matrix_transpose_mv_gpu(size_t rows, size_t cols,
                                   BufferView<const Real> block_values,
                                   BufferView<const int> block_row_ptrs,
                                   BufferView<const int> block_col_indices,
                                   BufferView<const Real> rhs,
                                   BufferView<Real> dst, Real alpha, Real beta,
                                   std::shared_ptr<void> descr_type_erased) {
  cusparseContext *handle = get_cusparse_handle();

  size_t nnzb = block_values.Shape().Z();
  size_t block_size = block_values.Shape().X();
  if (block_size > 1) {
    cusparseMatDescr_t descr =
        static_cast<cusparseMatDescr_t>(descr_type_erased.get());
    auto status = cusparseDbsrmv(
        handle, CUSPARSE_DIRECTION_COLUMN, CUSPARSE_OPERATION_TRANSPOSE, rows,
        cols, nnzb, &alpha, descr, block_values.Data(), block_row_ptrs.Data(),
        block_col_indices.Data(), block_size, rhs.Data(), &beta, dst.Data());

    if (status != CUSPARSE_STATUS_SUCCESS) {
      AX_THROW_RUNTIME_ERROR("cusparseDbsrmm failed {}: {}",
                             cusparseGetErrorName(status),
                             cusparseGetErrorString(status));
    }
  } else {
    // CSR matrix
    compute_csr_spmv_transpose_gpu(flatten(rhs), flatten(dst), alpha, beta,
                                   descr_type_erased);
  }
}

std::shared_ptr<void> create_bsr_mat_desc(BufferView<int> row_ptrs,
                                          BufferView<int> col_indices,
                                          BufferView<Real> values, size_t rows,
                                          size_t cols) {
  size_t bs = values.Shape().X();

  if (bs > 1) {
    cusparseMatDescr_t descr;
    cusparseCreateMatDescr(&descr);
    cusparseSetMatType(descr, CUSPARSE_MATRIX_TYPE_GENERAL);
    cusparseSetMatIndexBase(descr, CUSPARSE_INDEX_BASE_ZERO);
    return std::shared_ptr<void>(descr, [](void *descr) {
      cusparseDestroyMatDescr(static_cast<cusparseMatDescr_t>(descr));
    });
  } else {
    // CSR matrix
    return create_csr_compress_desc_gpu(row_ptrs, col_indices, flatten(values),
                                        rows, cols);
  }
}

} // namespace ax::math::details