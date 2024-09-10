#pragma once
#include "buffer.hpp"

namespace ax {
namespace details {
template <typename T>
AX_HOST_DEVICE AX_FORCE_INLINE T* buffer_at(T* data, Dim3 const& shape, Dim3 const& strides,
                                            size_t x, size_t y, size_t z) {
  assert(x < shape.X() && (shape.Y() == 0 || y < shape.Y()) && (shape.Z() == 0 || z < shape.Z()));
  AX_UNUSED(shape);
  using CharT = std::conditional_t<std::is_const_v<T>, const char, char>;
  CharT* base = reinterpret_cast<CharT*>(data);
  return reinterpret_cast<T*>(base + details::dot_prod(strides, Dim3{x, y, z}));
}

template <typename T>
struct BufferViewIterator {
  BufferViewIterator();

  using value_type = std::remove_const_t<T>;
  using reference = value_type&;
  using pointer = value_type*;

  BufferViewIterator(T* data, Dim3 const& shape, Dim3 const& strides)
      : data_(data), shape_(shape), strides_(strides) {}

  BufferViewIterator(T* data, Dim3 const& shape, Dim3 const& strides, Dim3 const& current)
      : data_(data), current_(current), shape_(shape), strides_(strides) {}

  AX_FORCE_INLINE T& operator*() noexcept {
    return *buffer_at(data_, shape_, strides_, current_.X(), current_.Y(), current_.Z());
  }

  AX_FORCE_INLINE T& operator*() const noexcept {
    return *buffer_at(data_, shape_, strides_, current_.X(), current_.Y(), current_.Z());
  }

  BufferViewIterator& operator++() {
    if (current_.X() + 1 < shape_.X()) {
      ++current_.X();
    } else if (current_.Y() + 1 < shape_.Y()) {
      current_.X() = 0;
      ++current_.Y();
    } else {
      current_.X() = 0;
      current_.Y() = 0;
      ++current_.Z();
    }
    return *this;
  }

  BufferViewIterator operator++(int) {
    BufferViewIterator tmp = *this;
    ++(*this);
    return tmp;
  }

  bool operator==(BufferViewIterator const& other) const { return current_ == other.current_; }

  bool operator!=(BufferViewIterator const& other) const { return !(*this == other); }

  std::ptrdiff_t operator-(BufferViewIterator const& other) const {
    return details::linear_index(shape_, current_) - details::linear_index(shape_, other.current_);
  }

  T* data_{nullptr};
  Dim3 current_;
  Dim3 shape_;
  Dim3 strides_;
};

}  // namespace details

/**
 * @brief BufferView is a view of the buffer, it does not own the data, but only provide a view of
 *        the buffer. It is useful for CUDA, we can pass the view to the kernel, and the kernel can
 *        access the data directly.
 * @note  Notice that, the BufferView is not thread-safe, and it is user's duty to ensure the buffer
 *        is not destroyed before the view is used.
 * @note  The BufferView is valid for all continuous memory buffer, strides are in bytes. Although
 * we call it BufferView, it similar to Eigen::Map.
 *
 * @tparam T
 */
template <typename T>
class BufferView {
public:
  using ValueType = std::remove_const_t<T>;
  using ConstValueType = std::add_const_t<ValueType>;

  AX_HOST_DEVICE BufferView(T* data, Dim3 const& shape, Dim3 const& strides, BufferDevice device)
      : data_(data), device_(device), shape_(shape), strides_(strides) {}

  AX_HOST_DEVICE BufferView(T* data, Dim3 const& shape, BufferDevice device)
      : data_(data),
        device_(device),
        shape_(shape),
        strides_(details::compute_continuous_buffer_stride<T>(shape)) {}

  AX_HOST_DEVICE BufferView(T* data, size_t x, size_t y = 0, size_t z = 0)
      : BufferView(data, Dim3{x, y, z}) {}

  AX_HOST_DEVICE /* NOLINT */ operator BufferView<ConstValueType>() const {
    return {data_, shape_, strides_, device_};
  }

  AX_HOST_DEVICE BufferView(BufferView const& other) noexcept = default;
  AX_HOST_DEVICE BufferView& operator=(BufferView const& other) noexcept = default;
  AX_HOST_DEVICE BufferView(BufferView&& other) noexcept = default;
  AX_HOST_DEVICE BufferView& operator=(BufferView&& other) noexcept = default;

  /////////////////// buffer accessing ///////////////////
  AX_HOST_DEVICE AX_FORCE_INLINE T& operator()(size_t x) {
    assert(data_ != nullptr);
    return *details::buffer_at<T>(data_, shape_, strides_, x, 0, 0);
  }

  AX_HOST_DEVICE AX_FORCE_INLINE T const& operator()(size_t x) const {
    assert(data_ != nullptr);
    return *details::buffer_at<const T>(data_, shape_, strides_, x, 0, 0);
  }

  AX_HOST_DEVICE AX_FORCE_INLINE T& operator()(size_t x, size_t y) {
    assert(data_ != nullptr);
    return *details::buffer_at<T>(data_, shape_, strides_, x, y, 0);
  }

  AX_HOST_DEVICE AX_FORCE_INLINE T const& operator()(size_t x, size_t y) const {
    assert(data_ != nullptr);
    return *details::buffer_at<const T>(data_, shape_, strides_, x, y, 0);
  }

  AX_HOST_DEVICE AX_FORCE_INLINE T& operator()(size_t x, size_t y, size_t z) {
    assert(data_ != nullptr);
    return *details::buffer_at<T>(data_, shape_, strides_, x, y, z);
  }

  AX_HOST_DEVICE AX_FORCE_INLINE T const& operator()(size_t x, size_t y, size_t z) const {
    assert(data_ != nullptr);
    return *details::buffer_at<const T>(data_, shape_, strides_, x, y, z);
  }

  /////////////////// buffer accessing, raw ptr ///////////////////
  AX_HOST_DEVICE AX_FORCE_INLINE T* Offset(size_t x, size_t y = 0, size_t z = 0) {
    assert(data_ != nullptr);
    return details::buffer_at<T>(data_, shape_, strides_, x, y, z);
  }

  AX_HOST_DEVICE AX_FORCE_INLINE T const* Offset(size_t x, size_t y = 0, size_t z = 0) const {
    assert(data_ != nullptr);
    return details::buffer_at<const T>(data_, shape_, strides_, x, y, z);
  }

  /////////////////// subview ///////////////////
  AX_HOST_DEVICE AX_CONSTEXPR BufferView SubView(Dim3 const& shape);  // NOTE: not implemented for future...

  AX_HOST_DEVICE AX_CONSTEXPR Dim3 const& Shape() const { return shape_; }

  AX_HOST_DEVICE AX_CONSTEXPR Dim3 const& Stride() const { return strides_; }

  AX_HOST_DEVICE AX_CONSTEXPR T* Data() { return data_; }

  AX_HOST_DEVICE AX_CONSTEXPR const T* Data() const { return data_; }

  AX_HOST_DEVICE AX_CONSTEXPR BufferDevice Device() const { return device_; }

  AX_CONSTEXPR auto begin() /* NOLINT */ {
    return details::BufferViewIterator<T>(data_, shape_, strides_, {0, 0, 0});
  }

  AX_CONSTEXPR auto end() /* NOLINT */ {
    const Dim3 last = details::is_1d(shape_)   ? Dim3{shape_.X(), 0, 0}
                      : details::is_2d(shape_) ? Dim3{0, shape_.Y(), 0}
                                               : Dim3{0, 0, shape_.Z()};
    return details::BufferViewIterator<T>(data_, shape_, strides_, last);
  }

  AX_CONSTEXPR auto begin() const /* NOLINT */ {
    return details::BufferViewIterator<const T>(data_, shape_, strides_, {0, 0, 0});
  }

  AX_CONSTEXPR auto end() const /* NOLINT */ {
    const Dim3 last = details::is_1d(shape_)   ? Dim3{shape_.X(), 0, 0}
                      : details::is_2d(shape_) ? Dim3{0, shape_.Y(), 0}
                                               : Dim3{0, 0, shape_.Z()};
    return details::BufferViewIterator<const T>(data_, shape_, strides_, last);
  }

  AX_HOST_DEVICE AX_CONSTEXPR bool IsContinuous() const {
    return strides_ == details::compute_continuous_buffer_stride<T>(shape_);
  }

protected:
  T* data_{nullptr};
  BufferDevice device_;
  Dim3 shape_;
  Dim3 strides_;
};

template <typename T>
BufferView<T> view_from_raw_buffer(T* data, Dim3 const& shape, Dim3 const& strides,
                                   BufferDevice device = BufferDevice::Host) {
  return BufferView<T>(data, shape, strides, device);
}

template <typename T>
BufferView<T> view_from_raw_buffer(T* data, Dim3 const& shape,
                                   BufferDevice device = BufferDevice::Host) {
  return BufferView<T>(data, shape, device);
}

template <typename T, typename Alloc>
BufferView<T> view_from_buffer(std::vector<T, Alloc>& buffer) {
  return BufferView<T>(buffer.data(), Dim3{buffer.size()}, BufferDevice::Host);
}

template <typename T, typename Alloc>
BufferView<T> view_from_buffer(std::vector<T, Alloc>& buffer, Dim3 const& shape) {
  return BufferView<T>(buffer.data(), shape, BufferDevice::Host);
}

template <typename T, typename Alloc>
BufferView<T> view_from_buffer(std::vector<T, Alloc>& buffer, Dim3 const& shape,
                               Dim3 const& strides) {
  return BufferView<T>(buffer.data(), shape, strides, BufferDevice::Host);
}

template <typename T, typename Alloc>
BufferView<const T> view_from_buffer(const std::vector<T, Alloc>& buffer) {
  return BufferView<T>(buffer.data(), Dim3{buffer.size()}, BufferDevice::Host);
}

template <typename T, typename Alloc>
BufferView<const T> view_from_buffer(const std::vector<T, Alloc>& buffer, Dim3 const& shape) {
  return BufferView<T>(buffer.data(), shape, BufferDevice::Host);
}

template <typename T, typename Alloc>
BufferView<const T> view_from_buffer(const std::vector<T, Alloc>& buffer, Dim3 const& shape,
                                     Dim3 const& strides) {
  return BufferView<T>(buffer.data(), shape, strides, BufferDevice::Host);
}

using RealBufferView = BufferView<Real>;
using ConstRealBufferView = BufferView<const Real>;
using IndexBufferView = BufferView<Index>;
using ConstIndexBufferView = BufferView<const Index>;
using SizeBufferView = BufferView<size_t>;
using ConstSizeBufferView = BufferView<const size_t>;

}  // namespace ax