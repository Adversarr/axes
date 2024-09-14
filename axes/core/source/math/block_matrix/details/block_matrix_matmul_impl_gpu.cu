#include "ax/core/excepts.hpp"
#include "ax/math/block_matrix/details/matmul_impl.hpp"
#include "ax/math/details/cusparse_context.cuh"
#include <cusparse.h>

namespace ax::math::details {

void block_matrix_matmul_gpu(size_t rows, size_t cols,
                             BufferView<const Real> block_values,
                             BufferView<const int> block_row_ptrs,
                             BufferView<const int> block_col_indices,
                             BufferView<const Real> rhs, BufferView<Real> dst,
                             Real alpha, Real beta, void *descr_type_erased) {
  cusparseContext *handle = get_cusparse_handle();

  size_t nnzb = block_values.Shape().Z();
  size_t block_size = block_values.Shape().X();
  cusparseMatDescr_t descr = static_cast<cusparseMatDescr_t>(descr_type_erased);

  auto status = cusparseDbsrmv(
      handle, CUSPARSE_DIRECTION_COLUMN, CUSPARSE_OPERATION_NON_TRANSPOSE, rows,
      cols, nnzb, &alpha, descr, block_values.Data(), block_row_ptrs.Data(),
      block_col_indices.Data(), block_size, rhs.Data(), &beta, dst.Data());

  if (status != CUSPARSE_STATUS_SUCCESS) {
    throw make_runtime_error("cusparseDbsrmv failed {}: {}",
                             cusparseGetErrorName(status),
                             cusparseGetErrorString(status));
  }
}

std::shared_ptr<void> create_bsr_mat_desc_default() {
  cusparseMatDescr_t descr;
  cusparseCreateMatDescr(&descr);
  cusparseSetMatType(descr, CUSPARSE_MATRIX_TYPE_GENERAL);
  cusparseSetMatIndexBase(descr, CUSPARSE_INDEX_BASE_ZERO);
  return std::shared_ptr<void>(descr, [](void *descr) {
    cusparseDestroyMatDescr(static_cast<cusparseMatDescr_t>(descr));
  });
}

} // namespace ax::math::details