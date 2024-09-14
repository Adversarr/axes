#pragma once

#include <cusparse.h>
#include <stdexcept>
namespace ax::math::details {

struct CuSparseHandle {
  cusparseContext *handle_{nullptr};
  CuSparseHandle();
  ~CuSparseHandle();

  CuSparseHandle(CuSparseHandle const &) = delete;
  CuSparseHandle(CuSparseHandle &&other) {
    handle_ = other.handle_;
    other.handle_ = nullptr;
  }
};

// NOTE: must be called after ax::initialize.
cusparseContext *get_cusparse_handle();

} // namespace ax::math::details