#pragma once
#include "ax/core/common.hpp"

namespace ax::math {

template <size_t dim> struct ShapeArray {
  template <typename... Args,
            typename = std::enable_if_t<(std::is_integral_v<std::decay_t<Args>> && ...)>>
  explicit constexpr AX_FORCE_INLINE AX_HOST_DEVICE ShapeArray(Args const&... args)
      : data_{static_cast<size_t>(args)...} {
    static_assert(sizeof...(Args) == dim, "shape_array_t: dimension mismatch");
  }

  constexpr AX_HOST_DEVICE size_t operator[](size_t i) const noexcept { return data_[i]; }

  size_t data_[dim];
};

namespace details {

template <idx Idx, typename T> using project_t = T;

template <size_t dim> struct ShapeTupleImpl;
template <> struct ShapeTupleImpl<0> {
  using type = std::tuple<>;
};
template <size_t dim> struct ShapeTupleImpl {
  using type = decltype(std::tuple_cat(std::declval<typename ShapeTupleImpl<dim - 1>::type>(),
                                       std::declval<std::tuple<size_t>>()));
};

template <size_t dim> using ShapeTuple = typename ShapeTupleImpl<dim>::type;

template <size_t dim, size_t... Idx>
constexpr AX_HOST_DEVICE auto shape_tuple_to_array_impl(ShapeTuple<dim> const& tuple,
                                                        std::index_sequence<Idx...>)
    -> ShapeArray<dim> {
  return ShapeArray<dim>{std::get<Idx>(tuple)...};
}

template <size_t dim, size_t... Idx>
constexpr AX_HOST_DEVICE auto shape_array_to_tuple_impl(ShapeArray<dim> const& array,
                                                        std::index_sequence<Idx...>)
    -> ShapeTuple<dim> {
  return ShapeTuple<dim>{array[Idx]...};
}

template <size_t dim>
constexpr AX_HOST_DEVICE auto shape_tuple_to_array(ShapeTuple<dim> const& tuple) {
  return shape_tuple_to_array_impl(tuple, std::make_index_sequence<dim>{});
}

template <size_t dim>
constexpr AX_HOST_DEVICE auto shape_array_to_tuple(ShapeArray<dim> const& array) {
  return shape_array_to_tuple_impl(array, std::make_index_sequence<dim>{});
}

template <size_t dim, size_t... Idx> AX_HOST_DEVICE bool all_equal(ShapeArray<dim> const& a,
                                                                   ShapeArray<dim> const& b,
                                                                   std::index_sequence<Idx...>) {
  return ((a[Idx] == b[Idx]) && ...);
}

template <size_t dim, std::size_t... Idx>
AX_HOST_DEVICE size_t shape_size(ShapeArray<dim> const& shape_array, std::index_sequence<Idx...>) {
  static_assert(sizeof...(Idx) == dim, "shape_size: dimension mismatch");
  return (shape_array[Idx] * ...);
}

template <size_t dim, typename... SubT, std::size_t... Idx,
          typename = std::enable_if_t<(std::is_integral_v<std::decay_t<SubT>> && ...)>>
constexpr AX_HOST_DEVICE size_t sub2ind(ShapeArray<dim> const& stride_array, SubT const&... sub,
                                        std::index_sequence<Idx...>) {
  static_assert(sizeof...(Idx) == dim && sizeof...(Idx) == sizeof...(SubT),
                "sub2ind: dimension mismatch");
  return ((static_cast<size_t>(sub) * stride_array[Idx]) + ...);
}

template <size_t dim, size_t... Idx>
constexpr AX_HOST_DEVICE size_t sub2ind(ShapeArray<dim> const& shape_array,
                                        ShapeArray<dim> const& sub, std::index_sequence<Idx...>) {
  return ((shape_array[Idx] * sub[Idx]) + ...);
}

template <size_t dim, size_t i> struct StrideImpl {
  static constexpr AX_HOST_DEVICE size_t run(ShapeArray<dim> const& shape_array) noexcept {
    if constexpr (i >= dim - 1 || dim <= 1) {
      return 1;
    } else {
      return shape_array[i + 1] * StrideImpl<dim, i + 1>::run(shape_array);
    }
  }
};

template <size_t dim, size_t... Idx>
constexpr AX_HOST_DEVICE ShapeArray<dim> stride(ShapeArray<dim> const& shape_array,
                                                std::index_sequence<Idx...>) {
  return ShapeArray<dim>{StrideImpl<dim, Idx>::run(shape_array)...};
}

template <size_t dim, size_t... Idx>
constexpr AX_HOST_DEVICE bool is_valid_sub(ShapeArray<dim> const& shape, ShapeArray<dim> const& sub,
                                           std::index_sequence<Idx...>) {
  return ((sub[Idx] < shape[Idx]) && ...);
}

template <size_t dim, typename... SubT, size_t... Idx,
          typename = std::enable_if_t<(std::is_integral_v<std::decay_t<SubT>> && ...)>>
constexpr AX_HOST_DEVICE ShapeTuple<dim> ind2sub(ShapeArray<dim> const& extent, SubT const&... sub,
                                                 std::index_sequence<Idx...>) {
  return std::make_tuple(static_cast<size_t>(sub / extent[Idx])...);
}

template <size_t dim, size_t... Idx>
constexpr AX_HOST_DEVICE ShapeTuple<dim> ind2sub(ShapeArray<dim> const& extent,
                                                 ShapeArray<dim> const& stride, size_t ind,
                                                 std::index_sequence<Idx...>) {
  return std::make_tuple(static_cast<size_t>((ind / stride[Idx]) % extent[Idx])...);
}

}  // namespace details

// Implements most common indexing operations for multi-dimensional arrays.
// Note: Read-only.
template <size_t dim> class Shape {
public:
  using value_type = size_t;

  // constructors
  AX_CONSTEXPR AX_HOST_DEVICE explicit Shape(ShapeArray<dim> const& extent) noexcept
      : extent_(extent), stride_(details::stride<dim>(extent_, std::make_index_sequence<dim>{})) {}

  AX_CONSTEXPR AX_HOST_DEVICE Shape(Shape const&) noexcept = default;
  AX_CONSTEXPR AX_HOST_DEVICE Shape(Shape&&) noexcept = default;

  Shape& operator=(Shape const&) = delete;
  Shape& operator=(Shape&&) = delete;

  // shape arithmetic
  AX_CONSTEXPR AX_HOST_DEVICE size_t Size() const noexcept { return stride_[0] * extent_[0]; }
  AX_CONSTEXPR AX_HOST_DEVICE ShapeArray<dim> Extent() const noexcept { return extent_; }
  AX_CONSTEXPR AX_HOST_DEVICE ShapeArray<dim> Stride() const noexcept { return stride_; }
  // get
  AX_CONSTEXPR AX_HOST_DEVICE size_t Extent(size_t i) const noexcept { return extent_[i]; }
  AX_CONSTEXPR AX_HOST_DEVICE size_t Stride(size_t i) const noexcept { return stride_[i]; }
  AX_CONSTEXPR AX_HOST_DEVICE size_t operator[](size_t i) const noexcept { return Extent(i); }

  // indexing
  AX_CONSTEXPR AX_HOST_DEVICE size_t Sub2Ind(ShapeArray<dim> const& sub) const noexcept {
    return details::sub2ind<dim>(stride_, sub, std::make_index_sequence<dim>{});
  }

  template <typename... SubT,
            typename = std::enable_if_t<(std::is_integral_v<std::decay_t<SubT>> && ...)>>
  AX_CONSTEXPR AX_HOST_DEVICE size_t Sub2Ind(SubT const&... sub) const noexcept {
    return details::sub2ind<dim, SubT...>(stride_, sub..., std::make_index_sequence<dim>{});
  }

  AX_CONSTEXPR AX_HOST_DEVICE auto Ind2Sub(size_t ind) const noexcept {
    return details::ind2sub<dim>(extent_, stride_, ind, std::make_index_sequence<dim>{});
  }

  template <typename... SubT,
            typename = std::enable_if_t<(std::is_integral_v<std::decay_t<SubT>> && ...)>>
  AX_CONSTEXPR AX_HOST_DEVICE size_t operator()(SubT const&... sub) const noexcept {
    return Sub2Ind(sub...);
  }

  AX_CONSTEXPR AX_HOST_DEVICE bool IsValidSub(ShapeArray<dim> const& sub) const noexcept {
    return details::is_valid_sub(extent_, sub, std::make_index_sequence<dim>{});
  }

  template <typename... SubT,
            typename = std::enable_if_t<(std::is_integral_v<std::decay_t<SubT>> && ...)>>
  AX_CONSTEXPR AX_HOST_DEVICE bool IsValidSub(SubT const&... sub) const noexcept {
    return IsValidSub(ShapeArray<dim>{static_cast<size_t>(sub)...});
  }

private:
  const ShapeArray<dim> extent_;
  const ShapeArray<dim> stride_;
};

template <typename... Args,
          typename = std::enable_if_t<(std::is_integral_v<std::decay_t<Args>> && ...)>>
AX_CONSTEXPR AX_HOST_DEVICE auto make_shape(Args const&... args) {
  return Shape<sizeof...(Args)>(ShapeArray<sizeof...(Args)>{static_cast<size_t>(args)...});
}

}  // namespace ax::math
