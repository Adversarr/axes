#pragma once
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

#include "accessor.hpp"
#include "ax/core/cuda_memory.hpp"

namespace ax::math {

namespace details {

template <typename IndexType, int dim, size_t... Idx>
AX_FORCE_INLINE AX_HOST_DEVICE ShapeArray<IndexType, dim> compute_pitched_strides_impl(
    Shape<IndexType, dim> shape, std::index_sequence<Idx...>) noexcept {
  const auto& extent = shape.Extent();  // e.g.   1,  2, 3, 4
  const auto& stride = shape.Stride();  // e.g.  24, 12, 4, 1
  // result = 6, 4, 1, 0
  return ShapeArray<IndexType, dim>{stride[Idx] / extent[dim - 1]...,
                                    0};  // 0 is the last element of the array
}

template <typename IndexType, int dim>
AX_FORCE_INLINE AX_HOST_DEVICE ShapeArray<IndexType, dim> compute_pitched_strides(
    const Shape<IndexType, dim>& shape) noexcept {
  return compute_pitched_strides_impl<IndexType, dim>(
      shape, std::make_index_sequence<static_cast<size_t>(dim - 1)>());
}

template <typename... Args, typename Last>
auto last(Args&&..., Last&& last) -> decltype((std::forward<Last>(last))) {
  return (std::forward<Last>(last));
}

}  // namespace details

template <typename T>
struct BufferTraits<PitchedBuffer<T>> {
  using IndexType = size_t;
  using Borrowing = PitchedBuffer<T>;
  using ConstBorrowing = PitchedBuffer<T>;
  using Reference = T&;
  using ConstReference = const T&;
};

template <typename T, int dim, bool is_const>
class CudaPitchedAccessor : public FieldAccessorBase<CudaPitchedAccessor<T, dim, is_const>,
                                                     PitchedBuffer<T>, dim, is_const> {
public:
  static_assert(dim == 2 || dim == 3, "Pitched Ptr is only for 2D or 3D data");

  using Base
      = FieldAccessorBase<CudaPitchedAccessor<T, dim, is_const>, PitchedBuffer<T>, dim, is_const>;
  using Buffer = PitchedBuffer<T>;
  using ShapeType = typename Base::ShapeType;
  using IndexType = typename Base::IndexType;
  using Borrowing = typename Base::Borrowing;
  using ConstBorrowing = typename Base::ConstBorrowing;

  using Reference = typename Base::Reference;
  using ConstReference = typename Base::ConstReference;

  AX_FORCE_INLINE AX_HOST_DEVICE CudaPitchedAccessor(Borrowing buffer, ShapeType shape) noexcept
      : Base(buffer, shape),
        pitch_strides_(details::compute_pitched_strides<IndexType, dim>(shape)) {}

  AX_FORCE_INLINE AX_HOST_DEVICE CudaPitchedAccessor(const CudaPitchedAccessor& other) noexcept
      = default;
  AX_FORCE_INLINE AX_HOST_DEVICE CudaPitchedAccessor(CudaPitchedAccessor&& other) noexcept
      = default;

  template <typename... Args>
  AX_FORCE_INLINE AX_HOST_DEVICE Reference AtImpl(Args... args) noexcept {
    const ShapeArray<IndexType, dim> subscripts{std::forward<Args>(args)...};
    IndexType pitch_index = details::sub2ind<IndexType, dim>(pitch_strides_, subscripts,
                                                             std::make_index_sequence<dim>{});

    T* ptr = Base::data_.data(pitch_index);
    return *(ptr + subscripts[dim - 1]);
  }

  template <typename... Args>
  AX_FORCE_INLINE AX_HOST_DEVICE ConstReference AtImpl(Args... args) const noexcept {
    const ShapeArray<IndexType, dim> subscripts{std::forward<Args>(args)...};
    IndexType pitch_index = details::sub2ind<IndexType, dim>(pitch_strides_, subscripts,
                                                             std::make_index_sequence<dim>{});

    const T* ptr = Base::data_.data(pitch_index);
    return *(ptr + subscripts[dim - 1]);
  }

  AX_FORCE_INLINE AX_HOST_DEVICE IndexType GetTotalMemorySize() const noexcept {
    return Base::data_.total_size_;
  }

protected:
  ShapeArray<IndexType, dim> pitch_strides_;
};

template <typename T>
struct AccessorTypeFor<PitchedBuffer<T>> {
  template <int dim, bool is_const>
  using type = CudaPitchedAccessor<T, dim, is_const>;
};

// TODO: support for thrust
template <typename Tp, typename Alloc>
struct AccessorTypeFor<thrust::host_vector<Tp, Alloc>> {
  template <int dim, bool is_const>
  using type = FieldAccessorImplForStlLike<Tp, Alloc, dim, is_const, thrust::host_vector>;
};

template <typename Tp>
Span<Tp> make_span(thrust::device_vector<Tp>& vec) {
  return {thrust::raw_pointer_cast(vec.data()), vec.size()};
}

template <typename Tp>
Span<const Tp> make_span(const thrust::device_vector<Tp>& vec) {
  return {thrust::raw_pointer_cast(vec.data()), vec.size()};
}

}  // namespace ax::math
