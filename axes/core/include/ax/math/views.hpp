#pragma once
#include <range/v3/view/cartesian_product.hpp>
#include <range/v3/view/counted.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/zip.hpp>

#include "ax/math/accessor.hpp"
#include "shape.hpp"

namespace ax::math {

namespace details {

template <size_t dim, size_t... Idx>
auto make_shape_iterator(Shape<dim> const& shape, std::index_sequence<Idx...>) {
  return ranges::views::cartesian_product(
      ranges::views::iota(static_cast<size_t>(0), shape.Extent(Idx))...);
}

}  // namespace details

/**
 * @brief Iterates over a shape.
 * @tparam dim
 * @param shape
 * @return
 */
template <size_t dim> auto iter(Shape<dim> const& shape) {
  return details::make_shape_iterator(shape, std::make_index_sequence<dim>{});
}

template <typename T, size_t dim> auto enumerate(FieldAccessor<T, dim>& accessor) {
  return ranges::views::zip(
      iter(accessor.GetShape()),
      ranges::views::counted(accessor.Data(),
                             static_cast<std::ptrdiff_t>(accessor.GetShape().Size())));
}

template <typename T, size_t dim> auto enumerate(FieldAccessor<T, dim> const& accessor) {
  return ranges::views::zip(
      iter(accessor.GetShape()),
      ranges::views::counted(accessor.Data(),
                             static_cast<std::ptrdiff_t>(accessor.GetShape().Size())));
}

}  // namespace ax::math
