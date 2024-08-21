#pragma once
#include <cuda_runtime.h>

#include "excepts.hpp"
namespace ax {

template <typename T, int dim>
class RawCudaBuffer;

template <typename T>
struct PitchedBuffer {
  void* buffer_;
  size_t pitch_;      /**< Pitch of allocated memory in bytes */
  size_t total_size_; /**< Total size of allocated memory in bytes */

  AX_FORCE_INLINE AX_HOST_DEVICE T* data() noexcept { return static_cast<T*>(buffer_); }

  AX_FORCE_INLINE AX_HOST_DEVICE T* data(size_t pitch_index) noexcept {
    return reinterpret_cast<T*>(static_cast<char*>(buffer_) + pitch_index * pitch_);
  }

  AX_FORCE_INLINE AX_HOST_DEVICE const T* data() const noexcept {
    return static_cast<const T*>(buffer_);
  }

  AX_FORCE_INLINE AX_HOST_DEVICE const T* data(size_t pitch_index) const noexcept {
    return reinterpret_cast<const T*>(static_cast<const char*>(buffer_) + pitch_index * pitch_);
  }
};

template <typename T>
class RawCudaBuffer<T, 3> {
public:
  RawCudaBuffer() = default;

  explicit RawCudaBuffer(size_t width, size_t height, size_t depth) {
    cudaPitchedPtr p;
    auto res = cudaMalloc3D(&p, make_cudaExtent(width, height, depth));
    if (res != cudaSuccess) {
      ptr_ = nullptr;
      throw make_runtime_error("Failed to allocate cuda buffer: {}", cudaGetErrorString(res));
    }
    ptr_ = static_cast<T*>(p.ptr);
    pitch_ = p.pitch;
    extent_ = make_cudaExtent(width, height, depth);
  }

  ~RawCudaBuffer() {
    if (ptr_) {
      cudaFree(ptr_);
    }
  }

  // Disable copy
  RawCudaBuffer(RawCudaBuffer const&) = delete;
  RawCudaBuffer& operator=(RawCudaBuffer const&) = delete;

  // Enable move
  RawCudaBuffer(RawCudaBuffer&& other) noexcept
      : ptr_(other.ptr_), pitch_(other.pitch_), extent_(other.extent_) {
    other.ptr_ = nullptr;
    other.pitch_ = 0;
    other.extent_ = cudaExtent{0, 0, 0};
  }

  RawCudaBuffer& operator=(RawCudaBuffer&& other) noexcept {
    if (this == &other) {
      return *this;
    }

    RawCudaBuffer tmp(std::move(other));
    Swap(tmp);
    return *this;
  }

  void Swap(RawCudaBuffer& other) noexcept {
    std::swap(ptr_, other.ptr_);
    std::swap(pitch_, other.pitch_);
    std::swap(extent_, other.extent_);
  }

  cudaPitchedPtr ToCudaPitchedPtr() const noexcept {
    return make_cudaPitchedPtr(ptr_, pitch_, extent_.width, extent_.height);
  }

  PitchedBuffer<T> ToPitchedBuffer() const noexcept { return {ptr_, pitch_, Size()}; }

  size_t Size() const noexcept { return pitch_ * extent_.depth * extent_.height; }

protected:
  T* ptr_{nullptr};
  size_t pitch_{0};
  cudaExtent extent_{0, 0, 0};
};

template <typename T>
class RawCudaBuffer<T, 2> {
public:
  RawCudaBuffer() = default;
  RawCudaBuffer(size_t width, size_t height) {
    cudaPitchedPtr p;
    auto res = cudaMallocPitch(&p.ptr, &p.pitch, width * sizeof(T), height);
    if (res != cudaSuccess) {
      ptr_ = nullptr;
      throw make_runtime_error("Failed to allocate cuda buffer: {}", cudaGetErrorString(res));
    }
    ptr_ = static_cast<T*>(p.ptr);
    pitch_ = p.pitch;
    extent_ = make_cudaExtent(width * sizeof(T), height, 0);
  }

  ~RawCudaBuffer() {
    if (ptr_) {
      cudaFree(ptr_);
    }
  }

  // disable copy
  RawCudaBuffer(RawCudaBuffer const&) = delete;
  RawCudaBuffer& operator=(RawCudaBuffer const&) = delete;

  // enable move
  RawCudaBuffer(RawCudaBuffer&& other) noexcept
      : ptr_(other.ptr_), pitch_(other.pitch_), extent_(other.extent_) {
    other.ptr_ = nullptr;
    other.pitch_ = 0;
    other.extent_ = cudaExtent{0, 0, 0};
  }
  RawCudaBuffer& operator=(RawCudaBuffer&& other) noexcept {
    if (this == &other) {
      return *this;
    }

    RawCudaBuffer tmp(std::move(other));
    Swap(tmp);
    return *this;
  }


  void Swap(RawCudaBuffer& other) noexcept {
    std::swap(ptr_, other.ptr_);
    std::swap(pitch_, other.pitch_);
    std::swap(extent_, other.extent_);
  }

  cudaPitchedPtr ToCudaPitchedPtr() const noexcept {
    return make_cudaPitchedPtr(ptr_, pitch_, extent_.width, extent_.height);
  }

  PitchedBuffer<T> ToPitchedBuffer() const noexcept { return {ptr_, pitch_, Size()}; }

  size_t Size() const noexcept { return pitch_ * extent_.height; }


protected:
  T* ptr_{nullptr};
  size_t pitch_{0};
  cudaExtent extent_{0, 0, 0};
};

}  // namespace ax