#pragma once
#include <range/v3/view/cartesian_product.hpp>
#include <range/v3/view/counted.hpp>
#include <range/v3/view/indirect.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>

#include "ax/math/accessor.hpp"
#include "shape.hpp"

namespace ax::math {

namespace details {

template <typename IndexType, int dim, size_t... Idx>
auto make_shape_iterator(Shape<IndexType, dim> const& shape, std::index_sequence<Idx...>) {
  return ranges::views::cartesian_product(
      ranges::views::ints(static_cast<IndexType>(0), shape.Extent(Idx))...);
}

template <int dim, typename IndexType, typename... Args, size_t... Idx>
ShapeArray<IndexType, dim> cast_shape_array_impl(
    ranges::common_tuple<IndexType, Args...> const& tuple, std::index_sequence<Idx...>) {
  return ShapeArray<IndexType, dim>(std::get<Idx>(tuple)...);
}

template <int dim, typename IndexType, typename... Args,
          typename = std::enable_if_t<(std::is_same_v<IndexType, Args> && ...)>>
ShapeArray<IndexType, dim> cast_shape_array(ranges::common_tuple<IndexType, Args...> const& tuple) {
  return cast_shape_array_impl<dim, IndexType, Args...>(
      tuple, std::make_index_sequence<sizeof...(Args) + 1>{});
}

template <typename IndexType, int dim>
struct ShapeCursor {
  using value_type = ShapeArray<IndexType, dim>;
  using difference_type = std::ptrdiff_t;
  using ShapeArrayT = ShapeArray<IndexType, dim>;
  using ShapeT = Shape<IndexType, dim>;

  struct mixin;

  ShapeCursor() = default;
  explicit ShapeCursor(const ShapeT& shape) : shape_{shape} {}
  ShapeCursor(const ShapeT& shape, const ShapeArrayT& cursor) : cursor_{cursor}, shape_{shape} {}
  ShapeCursor(const ShapeCursor&) noexcept = default;
  ShapeCursor& operator=(const ShapeCursor&) noexcept = default;
  ShapeCursor(ShapeCursor&&) noexcept = default;
  ShapeCursor& operator=(ShapeCursor&&) noexcept = default;

  value_type read() const { return cursor_; }

  bool equal(const ShapeCursor& other) const {
    return details::equal<IndexType, dim>(cursor_, other.cursor_);
  }

  template <size_t sub>
  void next_(std::integral_constant<size_t, sub>) {
    if (cursor_[sub] + 1 < shape_.Extent(sub)) {
      ++(cursor_[sub]);
    } else {
      cursor_[sub] = 0;
      next_(std::integral_constant<size_t, sub - 1>{});
    }
  }

  void next_(std::integral_constant<size_t, 0>) { ++(cursor_[0]); }

  template <size_t sub>
  void prev_(std::integral_constant<size_t, sub>) {
    if (cursor_[sub] > 0) {
      --(cursor_[sub]);
    } else {
      cursor_[sub] = shape_.Extent(sub) - 1;
      prev_(std::integral_constant<size_t, sub - 1>{});
    }
  }

  void prev_(std::integral_constant<size_t, 0>) { --(cursor_[0]); }

  void next() { next_(std::integral_constant<size_t, dim - 1>{}); }

  void prev() { prev_(std::integral_constant<size_t, dim - 1>{}); }

  void advance(difference_type n) {
    // how much should we advance in this dimension?
    const auto current_ind = shape_.Sub2Ind(cursor_);
    const auto max_ind = static_cast<difference_type>(shape_.Size());
    const auto required_ind = current_ind + n;
    assert(current_ind >= 0 && current_ind < max_ind);
    cursor_ = shape_.Ind2Sub(required_ind);
  }

  std::ptrdiff_t distance_to(const ShapeCursor& other) const {
    auto current_ind = static_cast<difference_type>(shape_.Sub2Ind(cursor_));
    auto other_ind = static_cast<difference_type>(shape_.Sub2Ind(other.cursor_));

    return other_ind - current_ind;
  }

  ShapeArrayT cursor_;
  ShapeT shape_;
};

template <typename IndexType, int dim>
struct ShapeCursor<IndexType, dim>::mixin : ranges::basic_mixin<ShapeCursor<IndexType, dim>> {
  using ranges::basic_mixin<ShapeCursor<IndexType, dim>>::basic_mixin;
  using ShapeT = Shape<IndexType, dim>;
  using ShapeArrayT = ShapeArray<IndexType, dim>;

  explicit mixin(ShapeT const& shape) : mixin{ShapeCursor<IndexType, dim>(shape)} {}
  mixin(ShapeT const& shape, ShapeArrayT const& cursor)
      : mixin{ShapeCursor<IndexType, dim>(shape, cursor)} {}
};

template <typename IndexType>
struct ShapeCursor<IndexType, 1> {
  using value_type = IndexType;
  using difference_type = std::ptrdiff_t;
  using ShapeArrayT = ShapeArray<IndexType, 1>;
  using ShapeT = Shape<IndexType, 1>;

  struct mixin;

  ShapeCursor() = default;
  explicit ShapeCursor(const ShapeT& shape) : cursor_{0}, shape_{shape} {}
  ShapeCursor(const ShapeT& shape, const ShapeArrayT& cursor)
      : cursor_{cursor.template get<0>()}, shape_{shape} {}
  ShapeCursor(const ShapeCursor&) noexcept = default;
  ShapeCursor& operator=(const ShapeCursor&) noexcept = default;
  ShapeCursor(ShapeCursor&&) noexcept = default;
  ShapeCursor& operator=(ShapeCursor&&) noexcept = default;

  value_type read() const { return cursor_; }

  bool equal(const ShapeCursor& other) const { return cursor_ == other.cursor_; }

  void next() { ++cursor_; }

  void prev() { --cursor_; }

  void advance(difference_type n) { cursor_ += n; }

  std::ptrdiff_t distance_to(const ShapeCursor& other) const {
    return static_cast<std::ptrdiff_t>(other.cursor_) - static_cast<std::ptrdiff_t>(cursor_);
  }

  IndexType cursor_;
  ShapeT shape_;
};

template <typename IndexType>
struct ShapeCursor<IndexType, 1>::mixin : ranges::basic_mixin<ShapeCursor<IndexType, 1>> {
  using ranges::basic_mixin<ShapeCursor<IndexType, 1>>::basic_mixin;
  using ShapeT = Shape<IndexType, 1>;
  using ShapeArrayT = ShapeArray<IndexType, 1>;

  explicit mixin(ShapeT const& shape) : mixin{ShapeCursor<IndexType, 1>(shape)} {}
  mixin(ShapeT const& shape, ShapeArrayT const& cursor)
      : mixin{ShapeCursor<IndexType, 1>(shape, cursor)} {}
};

template <typename IndexType, int dim>
using ShapeIterator = ranges::basic_iterator<ShapeCursor<IndexType, dim>>;

}  // namespace details

using details::ShapeIterator;

/**
 * @brief Iterates over a shape.
 * @tparam dim
 * @param shape
 * @return
 */
template <typename IndexType, int dim>
auto iter(Shape<IndexType, dim> const& shape) {
  // return details::make_shape_iterator(shape, std::make_index_sequence<dim>{});
  return ranges::views::iota(
             ShapeIterator<IndexType, dim>(shape),
             ShapeIterator<IndexType, dim>(shape, details::unit<IndexType, dim>(shape.Extent(0))))
         | ranges::views::indirect;
}

template <typename Derived, typename Buffer, int dim, bool is_const>
auto enumerate(FieldAccessorBase<Derived, Buffer, dim, is_const>& accessor) {
  using namespace ranges::views;
  return zip(iter(accessor.GetShape()),
             iota(accessor.AsDerived().begin(), accessor.AsDerived().end()) | indirect);
}

template <typename Derived, typename Buffer, int dim, bool is_const>
auto enumerate(FieldAccessorBase<Derived, Buffer, dim, is_const> const& accessor) {
  using namespace ranges::views;
  return zip(iter(accessor.GetShape()),
             iota(accessor.AsDerived().begin(), accessor.AsDerived().end()) | indirect);
}

template <typename IndexType, int dim>
auto begin(Shape<IndexType, dim> const& shape) {
  return ShapeIterator<IndexType, dim>(shape);
}

template <typename IndexType, int dim>
auto end(Shape<IndexType, dim> const& shape) {
  return ShapeIterator<IndexType, dim>(shape, details::unit<IndexType, dim>(shape.Extent(0)));
}

}  // namespace ax::math

// Extensions

namespace ranges {

// template <typename Container, int dim, bool is_const>
// auto begin(ax::math::FieldAccessor<Container, dim, is_const>& accessor) {
//   return accessor.begin();
// }
//
// template <typename Container, int dim, bool is_const>
// auto end(ax::math::FieldAccessor<Container, dim, is_const>& accessor) {
//   return accessor.end();
// }

}  // namespace ranges

// namespace std {
//
// template <typename Container, int dim, bool is_const>
// auto begin(ax::math::FieldAccessor<Container, dim, is_const>& accessor) {
//   return accessor.begin();
// }
//
// template <typename Container, int dim, bool is_const>
// auto end(ax::math::FieldAccessor<Container, dim, is_const>& accessor) {
//   return accessor.end();
// }
//
// template <typename Container, int dim, bool is_const>
// auto cbegin(ax::math::FieldAccessor<Container, dim, is_const> const& accessor) {
//   return accessor.begin();
// }
//
// template <typename Container, int dim, bool is_const>
// auto cend(ax::math::FieldAccessor<Container, dim, is_const> const& accessor) {
//   return accessor.end();
// }
//
// }  // namespace std