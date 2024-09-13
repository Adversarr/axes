#include "ax/core/excepts.hpp"
#include "ax/math/block_matrix/details/cusparse_context.cuh"
#include "csr_compress_impl.hpp"
#include <cusparse.h>

namespace ax::math::details {

struct Descr {
  cusparseSpMatDescr_t descr_{nullptr};
  cusparseDnVecDescr_t rhs_descr_{nullptr};
  cusparseDnVecDescr_t lhs_descr_{nullptr};
  cusparseDnVecDescr_t rhs_dst_descr_{nullptr};
  cusparseDnVecDescr_t lhs_dst_descr_{nullptr};

  Real *buffer_{nullptr};
  size_t buffer_size_{0};

  Descr(size_t rows, size_t cols, size_t nnz, int *row_ptrs, int *col_indices,
        Real *values) {
    auto status =
        cusparseCreateCsr(&descr_, rows, cols, nnz, row_ptrs, col_indices,
                          values, CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I,
                          CUSPARSE_INDEX_BASE_ZERO, CUDA_R_64F);

    if (status != CUSPARSE_STATUS_SUCCESS) {
      throw make_runtime_error("Failed to create CSR matrix descriptor {}: {}", 
        cusparseGetErrorName(status), cusparseGetErrorString(status));
    }

    status = cusparseCreateDnVec(&rhs_descr_, cols, nullptr, CUDA_R_64F);
    if (status != CUSPARSE_STATUS_SUCCESS) {
      throw make_runtime_error("Failed to create dense vector descriptor {}: {}", 
        cusparseGetErrorName(status), cusparseGetErrorString(status));
    }

    status = cusparseCreateDnVec(&lhs_descr_, rows, nullptr, CUDA_R_64F);
    if (status != CUSPARSE_STATUS_SUCCESS) {
      throw make_runtime_error("Failed to create dense vector descriptor {}: {}", 
        cusparseGetErrorName(status), cusparseGetErrorString(status));
    }

    status = cusparseCreateDnVec(&rhs_dst_descr_, cols, nullptr, CUDA_R_64F);
    if (status != CUSPARSE_STATUS_SUCCESS) {
      throw make_runtime_error("Failed to create dense vector descriptor {}: {}", 
        cusparseGetErrorName(status), cusparseGetErrorString(status));
    }

    status = cusparseCreateDnVec(&lhs_dst_descr_, rows, nullptr, CUDA_R_64F);
    if (status != CUSPARSE_STATUS_SUCCESS) {
      throw make_runtime_error("Failed to create dense vector descriptor {}: {}", 
        cusparseGetErrorName(status), cusparseGetErrorString(status));
    }
  }

  ~Descr() {
    cusparseDestroySpMat(descr_);
    cusparseDestroyDnVec(rhs_descr_);
    cusparseDestroyDnVec(lhs_descr_);

    if (buffer_) {
      cudaFree(buffer_);
      buffer_ = nullptr;
    }
  }
};

std::shared_ptr<void> create_csr_compress_desc_gpu(BufferView<int> row_ptrs,
                                                   BufferView<int> col_indices,
                                                   BufferView<Real> values,
                                                   size_t rows, size_t cols) {
  return std::make_shared<Descr>(rows, cols, values.Shape().X(),
                                 row_ptrs.Data(), col_indices.Data(),
                                 values.Data());
}

void compute_csr_spmv_gpu(BufferView<const Real> x, BufferView<Real> y, Real alpha,
                          Real beta, std::shared_ptr<void> desc) {
  auto descr = std::static_pointer_cast<Descr>(desc);

  if (!is_1d(x.Shape())) {
    throw make_runtime_error("Input vector must be 1D for now.");
  }

  auto status = cusparseDnVecSetValues(descr->rhs_descr_, const_cast<Real*>(x.Data()));
  if (status != CUSPARSE_STATUS_SUCCESS) {
    throw make_runtime_error("Failed to set rhs dense vector values {}: {}",
                             cusparseGetErrorName(status),
                             cusparseGetErrorString(status));
  }

  status = cusparseDnVecSetValues(descr->rhs_dst_descr_, y.Data());
  if (status != CUSPARSE_STATUS_SUCCESS) {
    throw make_runtime_error("Failed to set dst dense vector values {}: {}",
                             cusparseGetErrorName(status),
                             cusparseGetErrorString(status));
  }

  auto handle = get_cusparse_handle();
  size_t required_buffer_size = 0;
  status = cusparseSpMV_bufferSize(
      handle, CUSPARSE_OPERATION_NON_TRANSPOSE, &alpha, descr->descr_,
      descr->rhs_descr_, &beta, descr->lhs_descr_, CUDA_R_64F,
      CUSPARSE_SPMV_ALG_DEFAULT, &required_buffer_size);
  if (status != CUSPARSE_STATUS_SUCCESS) {
    throw make_runtime_error("Failed to get buffer size {}: {}",
                             cusparseGetErrorName(status),
                             cusparseGetErrorString(status));
  }

  if (descr->buffer_ != nullptr) {
    if (descr->buffer_size_ < required_buffer_size) {
      cudaFree(descr->buffer_);
      descr->buffer_ = nullptr;
    }
  }

  if (descr->buffer_ == nullptr) {
    cudaMalloc(&descr->buffer_, required_buffer_size);
    descr->buffer_size_ = required_buffer_size;
  }

  status = cusparseSpMV_preprocess(handle, CUSPARSE_OPERATION_NON_TRANSPOSE,
                                   &alpha, descr->descr_, descr->rhs_descr_,
                                   &beta, descr->rhs_dst_descr_, CUDA_R_64F,
                                   CUSPARSE_SPMV_ALG_DEFAULT, descr->buffer_);
  if (status != CUSPARSE_STATUS_SUCCESS) {
    throw make_runtime_error("Failed to preprocess SPMV {}: {}",
                             cusparseGetErrorName(status),
                             cusparseGetErrorString(status));
  }

  status = cusparseSpMV(handle, CUSPARSE_OPERATION_NON_TRANSPOSE, &alpha,
                        descr->descr_, descr->rhs_descr_, &beta,
                        descr->rhs_dst_descr_, CUDA_R_64F,
                        CUSPARSE_SPMV_ALG_DEFAULT, descr->buffer_);
  if (status != CUSPARSE_STATUS_SUCCESS) {
    throw make_runtime_error("Failed to compute SPMV {}: {}",
                             cusparseGetErrorName(status),
                             cusparseGetErrorString(status));
  }
}

} // namespace ax::math::details