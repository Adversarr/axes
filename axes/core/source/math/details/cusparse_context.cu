#include "ax/core/entt.hpp"
#include "ax/core/logging.hpp"
#include "ax/math/details/cusparse_context.cuh"

namespace ax::math::details {

cusparseContext *get_cusparse_handle() {
  return ensure_resource<CuSparseHandle>().handle_;
}

CuSparseHandle::CuSparseHandle() {
  auto status = cusparseCreate(&handle_);
  AX_CHECK(status == CUSPARSE_STATUS_SUCCESS,
           "Failed to create cusparse handle, {}: {}",
           cusparseGetErrorName(status), cusparseGetErrorString(status));
}

CuSparseHandle::~CuSparseHandle() {
  if (handle_) {
    cusparseDestroy(handle_);
  }
  handle_ = nullptr;
}

} // namespace ax::math::details