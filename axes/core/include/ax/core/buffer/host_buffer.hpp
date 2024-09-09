#pragma once
#include "ax/core/buffer/buffer.hpp"

namespace ax {

template <typename T, typename Container = std::vector<T>>
class HostBuffer final : public Buffer<T> {
public:
  using Base = Buffer<T>;
  using PtrType = typename Base::PtrType;
  using ConstPtrType = typename Base::ConstPtrType;
  HostBuffer() = default;

  HostBuffer(HostBuffer&& other) noexcept : HostBuffer{} { Swap(other); }

  ~HostBuffer() noexcept override = default;

  void Swap(HostBuffer& other) noexcept {
    Base::SwapBaseData(other);
    container_.swap(other.container_);
  }

  void Resize(Dim3 const& shape) override {
    if (prod(shape) == container_.size()) {
      Buffer<T>::shape_ = shape;
      return;
    }

    // create a new and do swap, we do not need to maintain the old data.
    HostBuffer<T, Container> new_buffer(shape);
    new_buffer.Swap(*this);
  }

  PtrType Clone(const Dim3& new_shape) const override {
    if (prod(new_shape) == 0) {
      auto clone = Create(Buffer<T>::shape_);
      // std copy will be optimized by the compiler, no need to use memcpy.
      std::copy(container_.begin(), container_.end(), clone->container_.begin());
      return clone;
    } else {
      return Create(new_shape);
    }
  }

  void SetBytes(int value) override {
    std::memset(container_.data(), value, container_.size() * sizeof(T));
  }

  static std::unique_ptr<HostBuffer> Create(Dim3 const& shape) {
    return std::unique_ptr<HostBuffer>(new HostBuffer<T, Container>(shape));
  }

  Container& GetUnderlying() noexcept { return container_; }

  const Container& GetUnderlying() const noexcept { return container_; }

protected:
  Container container_;

  explicit HostBuffer(Dim3 const& shape)
      : Buffer<T>(nullptr, shape, BufferDevice::Host), container_(prod(shape)) {
    Buffer<T>::data_ = container_.data();
  }
};

}  // namespace ax
