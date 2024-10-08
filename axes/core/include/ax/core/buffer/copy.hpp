#pragma once
#include "buffer_view.hpp"
#include "ax/core/excepts.hpp"

#ifdef AX_HAS_CUDA
#  include <cuda_runtime_api.h>
#endif

namespace ax {
namespace details {

#ifdef AX_HAS_CUDA
template <typename From, typename To>
cudaError cu_copy(BufferView<const From> from, BufferView<To> to, cudaMemcpyKind kind) {
  auto s = cudaSuccess;
  if (from.IsContinuous() && to.IsContinuous()) {
    // For continuous buffer, just do copy
    const size_t physical_size = prod(from.Shape()) * sizeof(From);
    s = cudaMemcpy(to.Data(), from.Data(), physical_size, kind);
  } else {
    if constexpr (std::is_same_v<From, To>) {
      // check same shape
      if_unlikely (from.Shape() != to.Shape()) {
        AX_THROW_RUNTIME_ERROR(
            "Non-continuous CUDA copy between different shape is not invalid.");
      }
      if_unlikely (is_1d(from.Shape())) {
        AX_THROW_RUNTIME_ERROR("Non-continuous CUDA copy between 1D buffer is not supported.");
      }

      const auto [x, y, z] = *(from.Shape());

      const size_t width = x * sizeof(From);      //  Width of matrix transfer (columns in bytes)
      const size_t height = y * (z > 0 ? z : 1);  // Height of matrix transfer (rows)

      s = cudaMemcpy2D(to.Data(), to.Stride().Y(), from.Data(), from.Stride().Y(), width,
                       height, kind);
    } else {
      AX_THROW_RUNTIME_ERROR("CUDA copy between different types is not supported.");
    }
  }
  return s;
}
#endif

template <typename From, typename To>
void host_copy(BufferView<const From> from, BufferView<To> to) {
  if (from.IsContinuous() && to.IsContinuous()) {
    // Copy anyway.
    const size_t total_src_byte = prod(from.Shape()) * sizeof(From);
    std::memcpy(to.Data(), from.Data(), total_src_byte);
  } else {
    if constexpr (std::is_same_v<From, To>) {
      // Not continuous, we need from and to is same shape and type.
      if (from.Shape() != to.Shape()) {
        AX_THROW_RUNTIME_ERROR(
            "Non-continuous Host copy between different shape is not supported.");
      }

      // do copy.
      const size_t x_up = from.Shape().X();
      const size_t y_up = from.Shape().Y() > 0 ? from.Shape().Y() : 1;
      const size_t z_up = from.Shape().Z() > 0 ? from.Shape().Z() : 1;
      for (size_t z = 0; z < z_up; ++z) {
        for (size_t y = 0; y < y_up; ++y) {
          for (size_t x = 0; x < x_up; ++x) {
            to(x, y, z) = from(x, y, z);
          }
        }
      }
    } else {
      AX_THROW_RUNTIME_ERROR("Host copy between different types is not supported.");
    }
  }
}

template <typename From, typename To>
void copy_dispatch(BufferView<const From> from, BufferView<To> to) {
  if (from.Device() == BufferDevice::Host && to.Device() == BufferDevice::Host) {
    host_copy(from, to);
  } else {
#ifdef AX_HAS_CUDA
    cudaError err;
    if (from.Device() == BufferDevice::Device && to.Device() == BufferDevice::Device) {
      err = cu_copy(from, to, cudaMemcpyDeviceToDevice);
    } else if (from.Device() == BufferDevice::Host && to.Device() == BufferDevice::Device) {
      err = cu_copy(from, to, cudaMemcpyHostToDevice);
    } else /* if (from.Device() == BufferDevice::Device && to.Device() == BufferDevice::Host) */ {
      err = cu_copy(from, to, cudaMemcpyDeviceToHost);
    }
    if (err != cudaSuccess) {
      AX_THROW_RUNTIME_ERROR("CUDA copy failed {}: {}", cudaGetErrorName(err),
                             cudaGetErrorString(err));
    }
#else
    AX_CHECK(false, "CUDA is not enabled.");
    AX_UNREACHABLE();
#endif
  }
}

}  // namespace details

/**
 * @brief Copy data from one buffer to another.
 *
 * @tparam From
 * @tparam To
 * @param from
 * @param to
 */
template <typename To, typename From>
void copy(BufferView<To> to, BufferView<From> from) {
#ifdef AX_HAS_CUDA
  // check copiable.
  if (from.Device() == BufferDevice::Device || to.Device() == BufferDevice::Device) {
    if (!from.IsContinuous()) {
      AX_THROW_RUNTIME_ERROR("Copy between device buffer must be continuous.");
    }
    if (!to.IsContinuous()) {
      AX_THROW_RUNTIME_ERROR("Copy between device buffer must be continuous.");
    }
  }
#else
  AX_CHECK(from.Device() == BufferDevice::Host && to.Device() == BufferDevice::Host,
           "CUDA is not enabled.");
#endif
  const size_t total_src_byte = prod(from.Shape()) * sizeof(From);
  const size_t total_dst_byte = prod(to.Shape()) * sizeof(To);
  if_unlikely (total_src_byte > total_dst_byte) {
    AX_THROW_RUNTIME_ERROR("Buffer size not enough for copy. {} > {}", total_src_byte,
                             total_dst_byte);
  }

  details::copy_dispatch<std::remove_const_t<From>, To>(from, to);
}

}  // namespace ax