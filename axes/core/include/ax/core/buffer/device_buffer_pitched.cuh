#pragma once
#include "ax/utils/ensure_cuda_enabled.cuh"

#include "ax/core/buffer/buffer.hpp"
#include "ax/core/excepts.hpp"
#include <cuda_runtime_api.h>

namespace ax {

template <typename T> class DeviceBufferOptimized final : public Buffer<T> {
public:
  using Base = Buffer<T>;
  using PtrType = typename Base::PtrType;
  using ConstPtrType = typename Base::ConstPtrType;
  DeviceBufferOptimized() = default;

  DeviceBufferOptimized(DeviceBufferOptimized &&other) noexcept
      : DeviceBufferOptimized() {
    Swap(other);
  }

  ~DeviceBufferOptimized() {
    if (Base::data_) {
      cudaFree(Base::data_);
    }
    Base::data_ = nullptr;
  }

  void Swap(DeviceBufferOptimized &other) { Base::SwapBaseData(other); }

  void Resize(Dim3 const &shape) override {
    if (shape == Base::shape_) {
      // only do nothing if the shape is the same because of pitch memory
      return;
    }

    DeviceBufferOptimized<T> new_buffer(shape);
    Swap(new_buffer);
  }

  PtrType Clone(const Dim3 &new_shape) const {
    if (prod(new_shape) == 0) {
      auto new_buffer = Create(Base::shape_);
      // Copy between pitched ptr using cudaMemcpy is ok.
      cudaError error = cudaSuccess;
      if (is_1d(Base::shape_)) {
        error = cudaMemcpy(new_buffer->data_, Base::data_, Base::PhysicalSize(),
                           cudaMemcpyDeviceToDevice);
      } else {
        error = cudaMemcpy2D(new_buffer->data_, new_buffer->Stride().Y(),
                             Base::data_, Base::strides_.Y(),
                             Base::shape_.X() * sizeof(T), Base::shape_.Y(),
                             cudaMemcpyDeviceToDevice);
      }

      if (error != cudaSuccess) {
        throw make_runtime_error("Failed to copy device buffer: {}",
                                 cudaGetErrorName(error));
      }
      return new_buffer;
    } else {
      return Create(new_shape);
    }
  }

  static std::unique_ptr<DeviceBufferOptimized<T>> Create(Dim3 size) {
    return std::make_unique<DeviceBufferOptimized<T>>(size);
  }

  void SetBytes(int value) override {
    auto error = cudaMemset(Base::data_, value, Base::PhysicalSize());
    if (error != cudaSuccess) {
      throw make_runtime_error("Failed to copy device buffer: {}",
                               cudaGetErrorName(error));
    }
  }

  explicit DeviceBufferOptimized(Dim3 shape)
      : Base(nullptr, shape, BufferDevice::Device) {
    if (is_1d(shape)) {
      // create a 1d buffer
      allocate1d(shape.X());
      Base::strides_.X() = sizeof(T);
      Base::strides_.Y() = 0;
      Base::strides_.Z() = 0;
    } else if (is_2d(shape)) {
      // create a 2d buffer
      Base::strides_.X() = sizeof(T);
      Base::strides_.Y() = allocate2d(shape.X(), shape.Y());
      Base::strides_.Z() = 0;
    } else {
      // create a 3d buffer, a 3d buffer is stored as a 2d buffer, there is no
      // difference between the underlying logic between them.
      Base::strides_.X() = sizeof(T);
      Base::strides_.Y() = allocate2d(shape.X(), shape.Y() * shape.Z());
      Base::strides_.Z() = Base::strides_.Y() * shape.Y();
    }

    Base::shape_ = shape;
  }

protected:
  void allocate1d(size_t size) {
    void *data;
    auto result = cudaMalloc(&data, size * sizeof(T));
    if (result != cudaSuccess) {
      throw make_runtime_error("Failed to create device buffer: {}",
                               cudaGetErrorName(result));
    }
    Base::data_ = static_cast<T *>(data);
  }

  size_t allocate2d(size_t x, size_t y) {
    size_t pitch_x;
    void *data;
    auto result = cudaMallocPitch(&data, &pitch_x, x * sizeof(T), y);
    if (result != cudaSuccess) {
      throw make_runtime_error("Failed to create device buffer: {}",
                               cudaGetErrorName(result));
    }
    Base::data_ = static_cast<T *>(data);

    return pitch_x;
  }
};

} // namespace ax