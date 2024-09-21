#pragma once
#include "ax/core/excepts.hpp"
#include "host_buffer.hpp"
#ifdef AX_HAS_CUDA
#  include "device_buffer.cuh"
#endif

namespace ax {

namespace details {
#ifdef AX_HAS_CUDA
template <typename T>
using DefaultDeviceBuffer = DeviceBuffer<T>;
#endif
template <typename T>
using DefaultHostBuffer = HostBuffer<T>;

}  // namespace details

template <typename T>
BufferPtr<T> create_buffer(BufferDevice device, BufferDim shape) {
  if (device == BufferDevice::Device) {
#ifdef AX_HAS_CUDA
    return details::DefaultDeviceBuffer<T>::Create(shape);
#else
    AX_THROW_RUNTIME_ERROR("CUDA is not enabled.");
#endif
  } else {
    return details::DefaultHostBuffer<T>::Create(shape);
  }
}

template <typename T>
BufferPtr<T> ensure_buffer(BufferPtr<T> p, BufferDevice device, BufferDim shape) {
  if (p && p->Device() == device && p->Shape() == shape) {
    return p;
  }
  return create_buffer<T>(device, shape);
}

}  // namespace ax
