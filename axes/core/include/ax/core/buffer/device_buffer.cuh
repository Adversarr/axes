#pragma once
#include "ax/utils/ensure_cuda_enabled.cuh"

#include "ax/core/buffer/buffer.hpp"
#include "ax/core/excepts.hpp"
#include <cuda_runtime_api.h>

namespace ax {

template <typename T> class DeviceBuffer final : public Buffer<T> {
public:
  using Base = Buffer<T>;
  using PtrType = typename Base::PtrType;
  using ConstPtrType = typename Base::ConstPtrType;
  DeviceBuffer() = default;

  DeviceBuffer(DeviceBuffer &&other) noexcept : DeviceBuffer() { Swap(other); }

  ~DeviceBuffer() {
    if (Base::data_) {
      cudaFree(Base::data_);
    }
    Base::data_ = nullptr;
  }

  void Swap(DeviceBuffer &other) { Base::SwapBaseData(other); }

  void Resize(Dim3 const &shape) override {
    if (prod(shape) == prod(Base::shape_)) {
      Base::shape_ = shape;
      return;
    }

    DeviceBuffer<T> new_buffer(shape);
    Swap(new_buffer);
  }

  PtrType Clone(const Dim3 &new_shape) const override {
    if (prod(new_shape) == 0) {
      auto new_buffer = Create(Base::shape_);
      // Copy between pitched ptr using cudaMemcpy is ok.
      cudaError error = cudaSuccess;
      error = cudaMemcpy(new_buffer->data_, Base::data_, Base::PhysicalSize(),
                         cudaMemcpyDeviceToDevice);
      if (error != cudaSuccess) {
        throw make_runtime_error("Failed to copy device buffer: {}",
                                 cudaGetErrorName(error));
      }
      return new_buffer;
    } else {
      return Create(new_shape);
    }
  }

  static std::unique_ptr<DeviceBuffer<T>> Create(Dim3 size) {
    return std::make_unique<DeviceBuffer<T>>(size);
  }

  void SetBytes(int value) override {
    auto error = cudaMemset(Base::data_, value, Base::PhysicalSize());
    if (error != cudaSuccess) {
      throw make_runtime_error("Failed to copy device buffer: {}",
                               cudaGetErrorName(error));
    }
  }

  explicit DeviceBuffer(Dim3 shape)
      : Base(nullptr, shape, BufferDevice::Device) {
    // do malloc
    void *data;
    auto result = cudaMalloc(&data, prod(shape) * sizeof(T));
    if (result != cudaSuccess) {
      throw make_runtime_error("Failed to create device buffer: {}",
                               cudaGetErrorName(result));
    }

    Base::data_ = static_cast<T *>(data);
  }
};

} // namespace ax