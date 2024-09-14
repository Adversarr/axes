#include "ax/core/entt.hpp"
#include "ax/core/logging.hpp"
#include "ax/math/details/cublas_context.cuh"

namespace ax::math::details {

CuBlasContext::CuBlasContext() {
  auto status = cublasCreate_v2(&handle_);
  AX_CHECK(status == CUBLAS_STATUS_SUCCESS,
           "CuBlasContext: failed to create cublas handle, status: {}",
           cublasGetStatusString(status));
}

CuBlasContext::~CuBlasContext() {
  if (handle_) {
    cublasDestroy_v2(handle_);
  }
  handle_ = nullptr;
}

} // namespace ax::math::details