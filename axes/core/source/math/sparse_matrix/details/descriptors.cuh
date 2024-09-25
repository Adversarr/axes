#pragma once
#include "ax/core/excepts.hpp"
#include "ax/core/gsl.hpp"
#include "ax/math/details/cusparse_context.cuh"
#include <cusparse_v2.h>
#include "ax/utils/common.hpp"

#define CHECK_CUSPARSE(expr) do {\
  auto status = (expr);\
  if (status != CUSPARSE_STATUS_SUCCESS) {\
    AX_THROW_RUNTIME_ERROR("Failed to execute cusparse function {}: {}",\
                           cusparseGetErrorName(status),\
                           cusparseGetErrorString(status));\
  }\
} while (0)

#define CHECK_CUDA(expr) AX_EXPECTS((expr) == cudaSuccess)

namespace ax::math::details {

struct DnVec {
  explicit DnVec(int size) {
    auto *cusparse_handle = details::get_cusparse_handle();
    CHECK_CUSPARSE(cusparseCreateDnVec(&vec_, size, nullptr, CUDA_R_64F));
  }

  ~DnVec(){CHECK_CUSPARSE(cusparseDestroyDnVec(vec_));}

  AX_DECLARE_CONSTRUCTOR(DnVec, delete, delete);

  cusparseDnVecDescr_t vec_;
};

struct SpsvDescr {
  explicit SpsvDescr() {
    auto *cusparse_handle = details::get_cusparse_handle();
    CHECK_CUSPARSE(cusparseSpSV_createDescr(&spsvDescr_));
  }

  ~SpsvDescr(){CHECK_CUSPARSE(cusparseSpSV_destroyDescr(spsvDescr_));}

  AX_DECLARE_CONSTRUCTOR(SpsvDescr, delete, delete);

  cusparseSpSVDescr_t spsvDescr_;
};

struct CsrDescr {
  cusparseSpMatDescr_t sp_descr_{nullptr};
  cusparseDnVecDescr_t rhs_descr_{nullptr};
  cusparseDnVecDescr_t lhs_descr_{nullptr};
  cusparseDnVecDescr_t rhs_dst_descr_{nullptr};
  cusparseDnVecDescr_t lhs_dst_descr_{nullptr};

  Real *buffer_{nullptr};
  size_t buffer_size_{0};

  CsrDescr(size_t rows, size_t cols, size_t nnz, int *row_ptrs, int *col_indices,
        Real *values) {
    auto status =
        cusparseCreateCsr(&sp_descr_, rows, cols, nnz, row_ptrs, col_indices,
                          values, CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I,
                          CUSPARSE_INDEX_BASE_ZERO, CUDA_R_64F);

    if (status != CUSPARSE_STATUS_SUCCESS) {
      AX_THROW_RUNTIME_ERROR("Failed to create CSR matrix descriptor {}: {}",
                             cusparseGetErrorName(status),
                             cusparseGetErrorString(status));
    }

    status = cusparseCreateDnVec(&rhs_descr_, cols, nullptr, CUDA_R_64F);
    if (status != CUSPARSE_STATUS_SUCCESS) {
      AX_THROW_RUNTIME_ERROR("Failed to create dense vector descriptor {}: {}",
                             cusparseGetErrorName(status),
                             cusparseGetErrorString(status));
    }

    status = cusparseCreateDnVec(&lhs_descr_, rows, nullptr, CUDA_R_64F);
    if (status != CUSPARSE_STATUS_SUCCESS) {
      AX_THROW_RUNTIME_ERROR("Failed to create dense vector descriptor {}: {}",
                             cusparseGetErrorName(status),
                             cusparseGetErrorString(status));
    }

    status = cusparseCreateDnVec(&rhs_dst_descr_, rows, nullptr, CUDA_R_64F);
    if (status != CUSPARSE_STATUS_SUCCESS) {
      AX_THROW_RUNTIME_ERROR("Failed to create dense vector descriptor {}: {}",
                             cusparseGetErrorName(status),
                             cusparseGetErrorString(status));
    }

    status = cusparseCreateDnVec(&lhs_dst_descr_, cols, nullptr, CUDA_R_64F);
    if (status != CUSPARSE_STATUS_SUCCESS) {
      AX_THROW_RUNTIME_ERROR("Failed to create dense vector descriptor {}: {}",
                             cusparseGetErrorName(status),
                             cusparseGetErrorString(status));
    }
  }

  ~CsrDescr() {
    cusparseDestroySpMat(sp_descr_);
    cusparseDestroyDnVec(rhs_descr_);
    cusparseDestroyDnVec(lhs_descr_);

    if (buffer_) {
      cudaFree(buffer_);
      buffer_ = nullptr;
    }
  }
};

} // namespace ax::math::details