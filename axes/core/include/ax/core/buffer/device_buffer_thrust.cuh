#pragma once
#include "ax/utils/ensure_cuda_enabled.cuh"

#include "ax/core/buffer/buffer.hpp"
#include <cuda_runtime_api.h>
#include <thrust/device_vector.h>

namespace ax {

// A simpler way is use thrust to manage all the memory allocation and
// deallocation.
template <typename T> class DeviceBuffer final : public Buffer<T> {
public:
  using Base = Buffer<T>;
  using BasePtrType = typename Base::PtrType;
  using BaseConstPtrType = typename Base::ConstPtrType;
  DeviceBuffer() = default;

  DeviceBuffer(DeviceBuffer &&other) noexcept : DeviceBuffer() { Swap(other); }

  void Swap(DeviceBuffer &other) {
    data_.swap(other.data_);
    Base::SwapBaseData(other);
  }

  ~DeviceBuffer() = default;

  void Resize(const Dim3 &shape) override {
    if (prod(shape) == data_.size()) {
      Base::shape_ = shape;
      return;
    }
    DeviceBuffer<T> new_buffer(shape);
    Swap(new_buffer);
  }

  BasePtrType Clone(const Dim3 &new_shape) const override {
    if (prod(new_shape) == 0) {
      auto new_buffer = Create(Base::shape_);
      cudaMemcpy(thrust::raw_pointer_cast(new_buffer->data_.data()),
                 thrust::raw_pointer_cast(data_.data()), Base::PhysicalSize(),
                 cudaMemcpyDeviceToDevice);
      return new_buffer;
    } else {
      return Create(new_shape);
    }
  }

  static std::unique_ptr<DeviceBuffer<T>> Create(Dim3 size) {
    return std::make_unique<DeviceBuffer<T>>(size);
  }

  void SetBytes(int value) override {
    thrust::fill(data_.begin(), data_.end(), value);
  }

  thrust::device_vector<T> &GetUnderlying() { return data_; }
  const thrust::device_vector<T> &GetUnderlying() const { return data_; }

  explicit DeviceBuffer(Dim3 size)
      : Base(nullptr, size, BufferDevice::Device), data_(prod(size)) {
    Base::data_ = thrust::raw_pointer_cast(data_.data());
    Base::shape_ = size;
    Base::strides_ = details::compute_continuous_buffer_stride<T>(size);
  }

private:
  thrust::device_vector<T> data_;
};

} // namespace ax