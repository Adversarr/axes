#pragma once

#include <cusparse.h>
#include <stdexcept>
namespace ax::math::details {

struct CuSparseHandle {
  cusparseContext *handle_{nullptr};
  CuSparseHandle() {
    auto status = cusparseCreate(&handle_);
    if (status != CUSPARSE_STATUS_SUCCESS) {
      throw std::runtime_error("Failed to create cuSPARSE handle.");
    }
  }

  ~CuSparseHandle() {
    if (handle_) {
      cusparseDestroy(handle_);
    }
  }
};

// NOTE: must be called after ax::initialize.
cusparseContext *get_cusparse_handle();

} // namespace ax::math::details