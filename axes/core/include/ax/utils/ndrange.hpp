#pragma once
#include <range/v3/view/cartesian_product.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/stride.hpp>
#include <range/v3/view/zip.hpp>

#include "ax/core/common.hpp"
#include "ax/core/dim.hpp"
#include "ax/utils/dup_tuple.hpp"  // IWYU pragma: export
#include "ranges.hpp"              // IWYU pragma: export

/**
 * @namespace ax::utils
 * @brief Contains utility functions and aliases for the axes library.
 */
namespace ax {
namespace utils {

/****************************** range ******************************/

/**
 * @brief Generates a range of values from 0 to the specified end value.
 * @param end The end value of the range.
 * @return A range of values from 0 to end.
 */
template <typename IndexType, typename = std::enable_if_t<std::is_integral_v<IndexType>>>
AX_FORCE_INLINE auto range(IndexType end) {
  return views::iota(static_cast<IndexType>(0), end);
}

/**
 * @brief Generates a range of values from the specified begin value to the specified end value.
 * @param begin The begin value of the range.
 * @param end The end value of the range.
 * @return A range of values from begin to end.
 */
template <typename IndexType>
AX_FORCE_INLINE auto range(IndexType begin, IndexType end) {
  return views::iota(begin, end);
}

/**
 * @brief Generates a range of values from the specified begin value to the specified end value with
 * a specified step size.
 * @param begin The begin value of the range.
 * @param end The end value of the range.
 * @param step The step size between values in the range.
 * @return A range of values from begin to end with a step size of step.
 */
template <typename IndexType>
AX_FORCE_INLINE auto range(IndexType begin, IndexType end, IndexType step) {
  return views::iota(begin, end) | views::stride(step);
}

/**
 * @brief Generates a range of values from the first element of the specified index tuple to the
 * second element.
 * @param beg_end The index tuple specifying the begin and end values of the range.
 * @return A range of values from the first element of the index tuple to the second element.
 */
AX_FORCE_INLINE auto range(IndexTuple<2> const& beg_end) {
  auto [beg, end] = beg_end;
  return views::iota(beg, end);
}

/**
 * @brief Generates a range of values from the first element of the specified index tuple to the
 * second element with a specified step size.
 * @param beg_end_step The index tuple specifying the begin and end values of the range, and the
 * step size.
 * @return A range of values from the first element of the index tuple to the second element with a
 * step size of the third element.
 */
AX_FORCE_INLINE auto range(IndexTuple<3> const& beg_end_step) {
  auto [beg, end, step] = beg_end_step;
  return views::iota(beg, end) | views::stride(step);
}

/****************************** ndrange ******************************/

/**
 * @brief Generates a Cartesian product of multiple ranges of values.
 * @param args The ranges of values.
 * @return A Cartesian product of the specified ranges.
 */
template <typename IndexType, typename... T,
          typename = std::enable_if_t<(std::is_integral_v<std::remove_cvref_t<T>> && ...)>>
AX_FORCE_INLINE auto ndrange(T&&... args) {
  return views::cartesian_product(range(std::forward<T>(args))...);
}

AX_FORCE_INLINE auto ndrange(const Dim1& dim) {
  return range(dim.X());
}

AX_FORCE_INLINE AX_CONSTEXPR auto ndrange(const Dim2& dim) {
  return ndrange<size_t>(dim.X(), dim.Y());
}

AX_FORCE_INLINE AX_CONSTEXPR auto ndrange(const Dim3& dim) {
  return ndrange<size_t>(dim.X(), dim.Y(), dim.Z());
}
}  // namespace utils

}  // namespace ax
