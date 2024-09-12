#pragma once
#include "ax/core/logging.hpp"
#include "common.hpp"

namespace ax {

namespace details {

template <typename T>
AX_HOST_DEVICE AX_FORCE_INLINE Dim3 compute_continuous_buffer_stride(Dim3 const& shape) {
  Dim3 strides{0, 0, 0};
  strides.sizes_[0] = sizeof(T);
  strides.sizes_[1] = shape.Y() > 0 ? shape.X() * strides.X() : 0;
  strides.sizes_[2] = shape.Z() > 0 ? shape.Y() * strides.Y() : 0;
  return strides;
}

}  // namespace details

/**
 * @brief   Base class for both host and device buffer, assume continuous memory, but may have
 *          padding to support cuda pitch memory.
 *
 * @note    Virtual will not have large overhead, since these method are not frequently called.
 *
 * @warning It is user's duty to ensure that direct memory-copy between buffer will not hurt
 *          the underlying data, i.e. the copy assignment operator will not be called when the
 *          buffer is copied.
 *
 * @tparam T
 */
template <typename T>
class Buffer : std::enable_shared_from_this<T> {
public:
  using PtrType = std::shared_ptr<Buffer<T>>;  ///< Pointer type, must be shared for copy.
  using ConstPtrType = std::shared_ptr<const Buffer<T>>;  ///< Const pointer type.
  Buffer() = default;                                     // Should call derived 'Create' method.

  // Constructor, we allow you to create a buffer from a naive memory pointer, but it is your duty
  // to check the correctness of the memory.
  Buffer(T* data, Dim3 const& shape, BufferDevice device) noexcept
      : data_(data), device_(device), shape_(shape) {
    // The continuous memory buffer, without any padding or pitch.
    strides_ = details::compute_continuous_buffer_stride<T>(shape);
  }

  Buffer(T* data, Dim3 const& shape, Dim3 const& strides, BufferDevice device)
      : data_(data), device_(device), shape_(shape), strides_(strides) {
    AX_CHECK(strides_.X() >= sizeof(T),
             "Invalid strides.x, must larger than sizeof(T) for continuous memory.");
    AX_CHECK(strides_.Y() >= strides_.X() * shape_.X(),
             "Invalid strides.y, must larger than strides.x * shape.x for continuous memory.");
    AX_CHECK(strides_.Z() >= strides_.Y() * shape_.Y(),
             "Invalid strides.z, must larger than strides.y * shape.y for continuous memory.");
  }

  // Destructor, the base class do not know how to release the resource, its derived class's duty.
  virtual ~Buffer() noexcept = default;

  Buffer(Buffer const&) = delete;                 // no copy
  Buffer& operator=(Buffer const&) = delete;      // no copy
  Buffer(Buffer&&) noexcept = default;            // only move construct.
  Buffer& operator=(Buffer&&) noexcept = delete;  // no move assign.

  // Get the logical size of the buffer.
  size_t Size() const noexcept { return prod(shape_); }

  // Get the physical size of the buffer, count in bytes
  size_t PhysicalSize() const noexcept {
    if (shape_.Z() > 0) {
      return shape_.Z() * strides_.Z();
    } else if (shape_.Y() > 0) {
      return shape_.Y() * strides_.Y();
    } else {
      return shape_.X() * strides_.X();
    }
  }

  // Get the device of the buffer.
  BufferDevice Device() const noexcept { return device_; }

  //////////////////// Getter & Setter ////////////////////

  // Get the raw data pointer.
  T* Data() noexcept { return data_; }

  // Get the raw data pointer.
  const T* Data() const noexcept { return data_; }

  // Get the shape of the buffer.
  Dim3 const& Shape() const noexcept { return shape_; }

  // Get the strides of the buffer.
  Dim3 const& Stride() const noexcept { return strides_; }

  // Reserved for simplicity, but it is user's duty to guarantee you are not using a pitched ptr.
  T& operator[](size_t x) { return data_[x]; }

  T const& operator[](size_t x) const { return data_[x]; }

  // Clone this buffer, with same size, device, and data.
  virtual PtrType Clone(const Dim3& new_shape = {}) const = 0;  // NOLINT: google-default-arguments

  /////////////////// buffer size changing and direct settings ///////////////////
  // Resize, will check if the memory is enough, otherwise will throw an std::runtime_error.
  // valid both for 1D and 2D buffer.
  // WARN: Resizing a buffer will not keep the original data.
  void Resize(size_t x, size_t y = 1, size_t z = 1) { Resize(Dim3{x, y, z}); }

  virtual void Resize(Dim3 const& shape) = 0;
  virtual void SetBytes(int value) = 0;

  // NOTE: should move out of this class.
  // /////////////////// direct memory access ///////////////////
  // // Copy into another buffer, will assert the this size is smaller or equal to the target
  // buffer. virtual void CopyTo(Buffer<T>& other) const = 0;
  // // memset the buffer with value, typically zero

  // Assigning, just Resize then CopyFrom.
  void Assign(const T* data, size_t size, BufferDevice data_device) {
    Resize(size);
    CopyFrom(data, size, data_device);
  }

  // Get a view of the buffer, for GPU buffer, a shared ptr is not possible.
  BufferView<T> View() noexcept { return {data_, shape_, strides_, device_}; }

  // Get a view of the buffer, for GPU buffer, a shared ptr is not possible.
  BufferView<const T> View() const noexcept { return {data_, shape_, strides_, device_}; }

  BufferView<const T> ConstView() const noexcept { return {data_, shape_, strides_, device_}; }

  bool IsContinuous() const noexcept {
    return strides_ == details::compute_continuous_buffer_stride<T>(shape_);
  }

protected:
  void SwapBaseData(Buffer& other) noexcept {
    std::swap(data_, other.data_);
    std::swap(device_, other.device_);
    std::swap(shape_, other.shape_);
    std::swap(strides_, other.strides_);
  }

  T* data_{nullptr};
  BufferDevice device_{BufferDevice::Host};

  // atmost 3D buffer is supported. [x, y, z] here.
  // The x dimension is major, then y, z. For example, the layout of 2D buffer must be
  //    [(0, 0) (1, 0) (2, 0) ... (Sx-1, 0) (0, 1) (1, 1) ... (Sx-1, 1) ...]
  // Striding is also possible. (The default stride is (Sx, Sx * Sy, Sx * Sy * Sz))
  // The real index is computed as: (let sx = strides_[0], sy = strides_[1], sz = strides_[2])
  //    LogicalIndex = (x, y, z)
  //    PhysicalAddr = x + y * sx + z * sy
  // The stride value is always not used, but we preserve it to compute the actual memory cost:
  //    memory usage = sz * sizeof(T).
  Dim3 shape_{1, 1, 1};    // shape for each dimension.
  Dim3 strides_{0, 0, 0};  // strides for each dimension in bytes!
};

template <typename T>
using BufferPtr = typename Buffer<T>::PtrType;
template <typename T>
using BufferConstPtr = typename Buffer<T>::ConstPtrType;

template <typename BufferPtr>
auto make_view(BufferPtr&& buffer) {
  return buffer->View();
}

template <typename... BufferPtr>
auto make_view(BufferPtr&&... buffers) {
  return std::make_tuple(buffers->View()...);
}

}  // namespace ax