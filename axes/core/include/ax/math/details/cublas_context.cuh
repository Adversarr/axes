#pragma once
#include "ax/utils/ensure_cuda_enabled.cuh"
#include <cublas_v2.h>

namespace ax::math::details {

struct CuBlasContext {
  CuBlasContext();

  ~CuBlasContext();

  CuBlasContext(CuBlasContext const &) = delete;

  CuBlasContext(CuBlasContext &&other) {
    handle_ = other.handle_;
    other.handle_ = nullptr;
  }

  cublasHandle_t handle_;
};

} // namespace ax::math::details