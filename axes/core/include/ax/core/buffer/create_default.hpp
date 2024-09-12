#pragma once
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
#endif
  } else {
    return details::DefaultHostBuffer<T>::Create(shape);
  }
}

}  // namespace ax
