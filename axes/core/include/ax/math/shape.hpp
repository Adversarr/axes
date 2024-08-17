#pragma once
#include <array>
#include <tuple>

#include "ax/core/common.hpp"
#include "common.hpp"

namespace ax::math {

template <typename IndexType, int dim> struct ShapeArray {
  template <typename... Args, typename = std::enable_if_t<(std::is_integral_v<std::decay_t<Args>> && ...)>>
  explicit AX_CONSTEXPR AX_FORCE_INLINE AX_HOST_DEVICE ShapeArray(Args const&... args)
      : data_{static_cast<IndexType>(args)...} {
    static_assert(sizeof...(Args) == dim, "shape_array_t: dimension mismatch");
  }

  ShapeArray() : data_{0} {}

  ShapeArray(ShapeArray const&) noexcept = default;
  ShapeArray(ShapeArray&&) noexcept = default;
  ShapeArray& operator=(ShapeArray const&) noexcept = default;
  ShapeArray& operator=(ShapeArray&&) noexcept = default;

  AX_CONSTEXPR AX_HOST_DEVICE IndexType& operator[](size_t i) noexcept { return data_[i]; }
  AX_CONSTEXPR AX_HOST_DEVICE const IndexType& operator[](size_t i) const noexcept { return data_[i]; }

  template <size_t i> AX_CONSTEXPR AX_HOST_DEVICE IndexType& get() noexcept {
    static_assert(i < dim, "get: index out of range");
    return data_[i];
  }

  template <size_t i> AX_CONSTEXPR AX_HOST_DEVICE const IndexType& get() const noexcept {
    static_assert(i < dim, "get: index out of range");
    return data_[i];
  }

  IndexType data_[dim];
};

namespace details {

template <Index Idx, typename T> using project_t = T;

template <typename IndexType, int dim> struct ShapeTupleImpl;
template <typename IndexType> struct ShapeTupleImpl<IndexType, 0> {
  using type = std::tuple<>;
};
template <typename IndexType, int dim> struct ShapeTupleImpl {
  using type = decltype(std::tuple_cat(std::declval<typename ShapeTupleImpl<IndexType, dim - 1>::type>(),
                                       std::declval<std::tuple<IndexType>>()));
};

template <typename IndexType, int dim> using ShapeTuple = typename ShapeTupleImpl<IndexType, dim>::type;

template <typename IndexType, int dim, size_t... Idx>
AX_CONSTEXPR AX_HOST_DEVICE ShapeArray<IndexType, dim> shape_tuple_to_array_impl(
    ShapeTuple<IndexType, dim> const& tuple, std::index_sequence<Idx...>) {
  return ShapeArray<IndexType, dim>{std::get<Idx>(tuple)...};
}

template <typename IndexType, int dim, size_t... Idx>
AX_CONSTEXPR AX_HOST_DEVICE ShapeTuple<IndexType, dim> shape_array_to_tuple_impl(
    ShapeArray<IndexType, dim> const& array, std::index_sequence<Idx...>) {
  return ShapeTuple<IndexType, dim>{array[Idx]...};
}

template <typename IndexType, int dim>
AX_CONSTEXPR AX_HOST_DEVICE ShapeArray<IndexType, dim> shape_tuple_to_array(ShapeTuple<IndexType, dim> const& tuple) {
  return shape_tuple_to_array_impl<IndexType, dim>(tuple, std::make_index_sequence<dim>{});
}

template <typename IndexType, int dim>
AX_CONSTEXPR AX_HOST_DEVICE ShapeTuple<IndexType, dim> shape_array_to_tuple(ShapeArray<IndexType, dim> const& array) {
  return shape_array_to_tuple_impl(array, std::make_index_sequence<dim>{});
}

template <typename IndexType, int dim, size_t... Idx> AX_HOST_DEVICE bool all_equal(ShapeArray<IndexType, dim> const& a,
                                                                                    ShapeArray<IndexType, dim> const& b,
                                                                                    std::index_sequence<Idx...>) {
  return ((a[Idx] == b[Idx]) && ...);
}

template <typename IndexType, int dim>
bool equal(ShapeArray<IndexType, dim> const& lhs, ShapeArray<IndexType, dim> const& rhs) {
  return all_equal(lhs, rhs, std::make_index_sequence<dim>{});
}

template <typename IndexType, int dim, std::size_t... Idx>
AX_HOST_DEVICE size_t shape_size(ShapeArray<IndexType, dim> const& shape_array, std::index_sequence<Idx...>) {
  static_assert(sizeof...(Idx) == dim, "shape_size: dimension mismatch");
  return (shape_array[Idx] * ...);
}

template <typename IndexType, int dim, typename... SubT, std::size_t... Idx,
          typename = std::enable_if_t<(std::is_integral_v<std::decay_t<SubT>> && ...)>>
AX_CONSTEXPR AX_HOST_DEVICE IndexType sub2ind(ShapeArray<IndexType, dim> const& stride_array, SubT const&... sub,
                                              std::index_sequence<Idx...>) {
  static_assert(sizeof...(Idx) == dim && sizeof...(Idx) == sizeof...(SubT), "sub2ind: dimension mismatch");
  return ((static_cast<IndexType>(sub) * stride_array[Idx]) + ...);
}

template <typename IndexType, int dim, size_t... Idx>
AX_CONSTEXPR AX_HOST_DEVICE IndexType sub2ind(ShapeArray<IndexType, dim> const& shape_array,
                                              ShapeArray<IndexType, dim> const& sub, std::index_sequence<Idx...>) {
  return ((shape_array[Idx] * sub[Idx]) + ...);
}

template <typename IndexType, int dim, size_t i> struct StrideImpl {
  static AX_CONSTEXPR AX_HOST_DEVICE IndexType run(ShapeArray<IndexType, dim> const& shape_array) noexcept {
    if AX_CONSTEXPR (i >= dim - 1 || dim <= 1) {
      return 1;
    } else {
      return shape_array[i + 1] * StrideImpl<IndexType, dim, i + 1>::run(shape_array);
    }
  }
};

template <typename IndexType, int dim, size_t... Idx>
AX_CONSTEXPR AX_HOST_DEVICE ShapeArray<IndexType, dim> stride(ShapeArray<IndexType, dim> const& shape_array,
                                                              std::index_sequence<Idx...>) {
  return ShapeArray<IndexType, dim>(StrideImpl<IndexType, dim, Idx>::run(shape_array)...);
}

template <typename IndexType, int dim>
AX_CONSTEXPR AX_HOST_DEVICE ShapeArray<IndexType, dim> stride(const ShapeArray<IndexType, dim>& shape_array) {
  return stride(shape_array, std::make_index_sequence<dim>{});
}

template <typename IndexType, int dim, size_t... Idx>
AX_CONSTEXPR AX_HOST_DEVICE bool is_valid_sub(ShapeArray<IndexType, dim> const& shape,
                                              ShapeArray<IndexType, dim> const& sub, std::index_sequence<Idx...>) {
  return ((sub[Idx] < shape[Idx]) && ...);
}

template <typename IndexType, int dim, size_t... Idx>
AX_CONSTEXPR AX_HOST_DEVICE ShapeTuple<IndexType, dim> ind2sub_impl(ShapeArray<IndexType, dim> const& extent,
                                                                    ShapeArray<IndexType, dim> const& stride,
                                                                    size_t ind, std::index_sequence<Idx...>) {
  return std::make_tuple(static_cast<IndexType>((ind / stride[Idx]) % extent[Idx])...);
}

template <typename IndexType, int dim>
AX_CONSTEXPR AX_HOST_DEVICE ShapeArray<IndexType, dim> ind2sub(ShapeArray<IndexType, dim> const& extent,
                                                               ShapeArray<IndexType, dim> const& stride, size_t ind) {
  return shape_tuple_to_array<IndexType, dim>(ind2sub_impl(extent, stride, ind, std::make_index_sequence<dim>{}));
}

template <typename IndexType, typename AnotherIndexType, int dim, size_t... Idx>
AX_CONSTEXPR AX_HOST_DEVICE ShapeArray<IndexType, dim> cast_shape_array(
    ShapeArray<AnotherIndexType, dim> const& shape_array, std::index_sequence<Idx...>) {
  return ShapeArray<IndexType, dim>{static_cast<IndexType>(shape_array[Idx])...};
}

template <typename IndexType, typename AnotherIndexType, int dim>
AX_CONSTEXPR AX_HOST_DEVICE ShapeArray<IndexType, dim> cast_shape_array(
    ShapeArray<AnotherIndexType, dim> const& shape_array) {
  return cast_shape_array<IndexType>(shape_array, std::make_index_sequence<dim>{});
}

template <typename IndexType, int dim, size_t... Idx>
AX_CONSTEXPR AX_HOST_DEVICE ShapeArray<IndexType, dim> unit_impl(IndexType front, std::index_sequence<Idx...>) {
  return ShapeArray<IndexType, dim>{(Idx == 0 ? front : static_cast<IndexType>(0))...};
}

template <typename IndexType, int dim> AX_CONSTEXPR AX_HOST_DEVICE ShapeArray<IndexType, dim> unit(IndexType front) {
  return unit_impl<IndexType, dim>(front, std::make_index_sequence<dim>{});
}

}  // namespace details

template <typename IndexType, int dim> using ShapeTuple = details::ShapeTuple<IndexType, dim>;

// Implements most common indexing operations for multi-dimensional arrays.
// Note: Read-only.
template <typename IndexType, int dim> class Shape {
public:
  using ShapeArrayT = ShapeArray<IndexType, dim>;

  Shape() = default;

  // constructors
  AX_CONSTEXPR AX_HOST_DEVICE explicit Shape(ShapeArrayT const& extent) noexcept
      : extent_(extent), stride_(details::stride<IndexType, dim>(extent_, std::make_index_sequence<dim>{})) {}

  template <typename AnotherIndexType, typename = std::enable_if_t<std::is_same_v<IndexType, AnotherIndexType>>>
  Shape Cast() const noexcept {
    return *this;
  }

  template <typename AnotherIndexType, typename = std::enable_if_t<!std::is_same_v<IndexType, AnotherIndexType>>>
  Shape<AnotherIndexType, dim> Cast() const noexcept {
    return Shape<AnotherIndexType, dim>(
        details::cast_shape_array<AnotherIndexType>(extent_, std::make_index_sequence<dim>{}));
  }

  AX_CONSTEXPR AX_HOST_DEVICE Shape(Shape const&) noexcept = default;
  AX_CONSTEXPR AX_HOST_DEVICE Shape(Shape&&) noexcept = default;
  AX_CONSTEXPR AX_HOST_DEVICE Shape& operator=(Shape const&) noexcept = default;
  AX_CONSTEXPR AX_HOST_DEVICE Shape& operator=(Shape&&) noexcept = default;

  // shape arithmetic
  AX_CONSTEXPR AX_HOST_DEVICE IndexType Size() const noexcept { return stride_[0] * extent_[0]; }
  AX_CONSTEXPR AX_HOST_DEVICE ShapeArrayT Extent() const noexcept { return extent_; }
  AX_CONSTEXPR AX_HOST_DEVICE ShapeArrayT Stride() const noexcept { return stride_; }
  // get
  AX_CONSTEXPR AX_HOST_DEVICE IndexType Extent(size_t i) const noexcept { return extent_[i]; }
  AX_CONSTEXPR AX_HOST_DEVICE IndexType Stride(size_t i) const noexcept { return stride_[i]; }
  AX_CONSTEXPR AX_HOST_DEVICE IndexType operator[](size_t i) const noexcept { return Extent(i); }

  // indexing
  AX_CONSTEXPR AX_HOST_DEVICE IndexType Sub2Ind(ShapeArrayT const& sub) const noexcept {
    return details::sub2ind<IndexType, dim>(stride_, sub, std::make_index_sequence<dim>{});
  }

  template <typename... SubT, typename = std::enable_if_t<(std::is_integral_v<std::decay_t<SubT>> && ...)>>
  AX_CONSTEXPR AX_HOST_DEVICE IndexType Sub2Ind(SubT const&... sub) const noexcept {
    return details::sub2ind<IndexType, dim, SubT...>(stride_, sub..., std::make_index_sequence<dim>{});
  }

  AX_CONSTEXPR AX_HOST_DEVICE ShapeArrayT Ind2Sub(IndexType ind) const noexcept {
    return details::ind2sub<IndexType, dim>(extent_, stride_, ind);
  }

  template <typename... SubT, typename = std::enable_if_t<(std::is_integral_v<std::decay_t<SubT>> && ...)>>
  AX_CONSTEXPR AX_HOST_DEVICE IndexType operator()(SubT const&... sub) const noexcept {
    return Sub2Ind(sub...);
  }

  AX_CONSTEXPR AX_HOST_DEVICE bool IsValidSub(ShapeArrayT const& sub) const noexcept {
    return details::is_valid_sub(extent_, sub, std::make_index_sequence<dim>{});
  }

  template <typename... SubT, typename = std::enable_if_t<(std::is_integral_v<std::decay_t<SubT>> && ...)>>
  AX_CONSTEXPR AX_HOST_DEVICE bool IsValidSub(SubT const&... sub) const noexcept {
    return IsValidSub(ShapeArrayT{static_cast<IndexType>(sub)...});
  }

private:
  ShapeArrayT extent_;
  ShapeArrayT stride_;
};

template <typename IndexType = size_t, typename... Args,
          typename = std::enable_if_t<(std::is_integral_v<std::decay_t<Args>> && ...)>>
AX_CONSTEXPR AX_HOST_DEVICE auto make_shape(Args const&... args) {
  return Shape<IndexType, sizeof...(Args)>(ShapeArray<IndexType, sizeof...(Args)>{static_cast<size_t>(args)...});
}

template <typename IndexType, int dim>
AX_CONSTEXPR AX_HOST_DEVICE ShapeTuple<IndexType, dim> to_tuple(ShapeArray<IndexType, dim> const& shape_array) {
  return details::shape_array_to_tuple<IndexType, dim>(shape_array);
}

template <typename IndexType, int dim>
AX_CONSTEXPR AX_HOST_DEVICE ShapeArray<IndexType, dim> to_array(ShapeTuple<IndexType, dim> const& shape_tuple) {
  return details::shape_tuple_to_array<IndexType, dim>(shape_tuple);
}

template <typename IndexType, int dim> AX_CONSTEXPR AX_HOST_DEVICE Eigen::Map<const Vector<IndexType, dim>> to_vec(
    ShapeArray<IndexType, dim> const& shape_array) {
  return Eigen::Map<const Vector<IndexType, dim>>(shape_array.data_);
}

}  // namespace ax::math

namespace std {
template <typename IndexType, int dim> struct tuple_size<ax::math::ShapeArray<IndexType, dim>> {
  static constexpr size_t value = dim;
};

template <size_t I, typename IndexType, int dim> struct tuple_element<I, ax::math::ShapeArray<IndexType, dim>> {
  using type = IndexType;
};
}  // namespace std

// namespace ax::math {
//
// template <size_t I, typename IndexType, int dim> IndexType& get(ShapeArray<IndexType, dim>& arr) { return arr[I]; }
//
// template <size_t I, typename IndexType, int dim> const IndexType& get(const ShapeArray<IndexType, dim>& arr) {
//   return arr[I];
// }
//
// template <size_t I, typename IndexType, int dim> IndexType&& get(ShapeArray<IndexType, dim>&& arr) {
//   return std::move(get<I>(arr));
// }
//
// template <size_t I, typename IndexType, int dim> const IndexType&& get(const ShapeArray<IndexType, dim>&& arr) {
//   return std::move(get<I>(arr));
// }
//
// }  // namespace ax::math
