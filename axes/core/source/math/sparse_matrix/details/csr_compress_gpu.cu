#include "ax/core/excepts.hpp"
#include "ax/math/details/cusparse_context.cuh"
#include "csr_compress_impl.hpp"
#include <cusparse_v2.h>
#include "descriptors.cuh"

namespace ax::math::details {

std::shared_ptr<void> create_csr_compress_desc_gpu(IntBufferView row_ptrs,
                                                   IntBufferView col_indices,
                                                   BufferView<Real> values,
                                                   size_t rows, size_t cols) {
  return std::make_shared<CsrDescr>(rows, cols, values.Shape().X(),
                                 row_ptrs.Data(), col_indices.Data(),
                                 values.Data());
}

void compute_csr_spmv_gpu(BufferView<const Real> x, BufferView<Real> y,
                          Real alpha, Real beta, std::shared_ptr<void> desc) {
  auto descr = std::static_pointer_cast<CsrDescr>(desc);

  if (!is_1d(x.Shape())) {
    AX_THROW_RUNTIME_ERROR("Input vector must be 1D for now.");
  }

  auto status =
      cusparseDnVecSetValues(descr->rhs_descr_, const_cast<Real *>(x.Data()));
  if (status != CUSPARSE_STATUS_SUCCESS) {
    AX_THROW_RUNTIME_ERROR("Failed to set rhs dense vector values {}: {}",
                           cusparseGetErrorName(status),
                           cusparseGetErrorString(status));
  }

  status = cusparseDnVecSetValues(descr->rhs_dst_descr_, y.Data());
  if (status != CUSPARSE_STATUS_SUCCESS) {
    AX_THROW_RUNTIME_ERROR("Failed to set dst dense vector values {}: {}",
                           cusparseGetErrorName(status),
                           cusparseGetErrorString(status));
  }

  auto handle = get_cusparse_handle();
  size_t required_buffer_size = 0;
  status = cusparseSpMV_bufferSize(
      handle, CUSPARSE_OPERATION_NON_TRANSPOSE, &alpha, descr->sp_descr_,
      descr->rhs_descr_, &beta, descr->rhs_dst_descr_, CUDA_R_64F,
      CUSPARSE_SPMV_ALG_DEFAULT, &required_buffer_size);
  if (status != CUSPARSE_STATUS_SUCCESS) {
    AX_THROW_RUNTIME_ERROR("Failed to get buffer size {}: {}",
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
                                   &alpha, descr->sp_descr_, descr->rhs_descr_,
                                   &beta, descr->rhs_dst_descr_, CUDA_R_64F,
                                   CUSPARSE_SPMV_ALG_DEFAULT, descr->buffer_);
  if (status != CUSPARSE_STATUS_SUCCESS) {
    AX_THROW_RUNTIME_ERROR("Failed to preprocess SPMV {}: {}",
                           cusparseGetErrorName(status),
                           cusparseGetErrorString(status));
  }

  status = cusparseSpMV(handle, CUSPARSE_OPERATION_NON_TRANSPOSE, &alpha,
                        descr->sp_descr_, descr->rhs_descr_, &beta,
                        descr->rhs_dst_descr_, CUDA_R_64F,
                        CUSPARSE_SPMV_ALG_DEFAULT, descr->buffer_);
  if (status != CUSPARSE_STATUS_SUCCESS) {
    AX_THROW_RUNTIME_ERROR("Failed to compute SPMV {}: {}",
                           cusparseGetErrorName(status),
                           cusparseGetErrorString(status));
  }
}

void compute_csr_spmv_transpose_gpu(BufferView<const Real> x,
                                    BufferView<Real> y, Real alpha, Real beta,
                                    std::shared_ptr<void> desc) {
  auto descr = std::static_pointer_cast<CsrDescr>(desc);
  if (!is_1d(x.Shape())) {
    AX_THROW_RUNTIME_ERROR("Input vector must be 1D for now.");
  }

  auto status =
      cusparseDnVecSetValues(descr->lhs_descr_, const_cast<Real *>(x.Data()));
  if (status != CUSPARSE_STATUS_SUCCESS) {
    AX_THROW_RUNTIME_ERROR("Failed to set lhs dense vector values {}: {}",
                           cusparseGetErrorName(status),
                           cusparseGetErrorString(status));
  }

  status = cusparseDnVecSetValues(descr->lhs_dst_descr_, y.Data());
  if (status != CUSPARSE_STATUS_SUCCESS) {
    AX_THROW_RUNTIME_ERROR("Failed to set dst dense vector values {}: {}",
                           cusparseGetErrorName(status),
                           cusparseGetErrorString(status));
  }

  auto handle = get_cusparse_handle();
  size_t required_buffer_size = 0;
  status = cusparseSpMV_bufferSize(
      handle, CUSPARSE_OPERATION_TRANSPOSE, &alpha, descr->sp_descr_,
      descr->lhs_descr_, &beta, descr->lhs_dst_descr_, CUDA_R_64F,
      CUSPARSE_SPMV_ALG_DEFAULT, &required_buffer_size);
  if (status != CUSPARSE_STATUS_SUCCESS) {
    AX_THROW_RUNTIME_ERROR("Failed to get buffer size {}: {}",
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

  status = cusparseSpMV_preprocess(handle, CUSPARSE_OPERATION_TRANSPOSE, &alpha,
                                   descr->sp_descr_, descr->lhs_descr_, &beta,
                                   descr->lhs_dst_descr_, CUDA_R_64F,
                                   CUSPARSE_SPMV_ALG_DEFAULT, descr->buffer_);
  if (status != CUSPARSE_STATUS_SUCCESS) {
    AX_THROW_RUNTIME_ERROR("Failed to preprocess SPMV {}: {}",
                           cusparseGetErrorName(status),
                           cusparseGetErrorString(status));
  }

  status = cusparseSpMV(handle, CUSPARSE_OPERATION_TRANSPOSE, &alpha,
                        descr->sp_descr_, descr->lhs_descr_, &beta,
                        descr->lhs_dst_descr_, CUDA_R_64F,
                        CUSPARSE_SPMV_ALG_DEFAULT, descr->buffer_);
  if (status != CUSPARSE_STATUS_SUCCESS) {
    AX_THROW_RUNTIME_ERROR("Failed to compute SPMV {}: {}",
                           cusparseGetErrorName(status),
                           cusparseGetErrorString(status));
  }
}

} // namespace ax::math::details