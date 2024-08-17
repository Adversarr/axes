#pragma once
#include <range/v3/view.hpp>  // IWYU pragma: export

#include "ax/core/common.hpp"
#include "ax/utils/dup_tuple.hpp"  // IWYU pragma: export
#include "ranges.hpp"                // IWYU pragma: export

/**
 * @namespace ax::utils
 * @brief Contains utility functions and aliases for the axes library.
 */
namespace ax::utils {

/****************************** iota ******************************/

/**
 * @brief Generates a range of values from 0 to the specified end value.
 * @param end The end value of the range.
 * @return A range of values from 0 to end.
 */
AX_FORCE_INLINE auto iota(Index end) { return views::iota(static_cast<Index>(0), end); }

/**
 * @brief Generates a range of values from the specified begin value to the specified end value.
 * @param begin The begin value of the range.
 * @param end The end value of the range.
 * @return A range of values from begin to end.
 */
AX_FORCE_INLINE auto iota(Index begin, Index end) { return views::iota(begin, end); }

/**
 * @brief Generates a range of values from the specified begin value to the specified end value with a specified step size.
 * @param begin The begin value of the range.
 * @param end The end value of the range.
 * @param step The step size between values in the range.
 * @return A range of values from begin to end with a step size of step.
 */
AX_FORCE_INLINE auto iota(Index begin, Index end, Index step) {
  return views::iota(begin, end) | views::stride(step);
}

/**
 * @brief Generates a range of values from the first element of the specified index tuple to the second element.
 * @param be The index tuple specifying the begin and end values of the range.
 * @return A range of values from the first element of the index tuple to the second element.
 */
AX_FORCE_INLINE auto iota(Index_tuple<2> be) { return views::iota(std::get<0>(be), std::get<1>(be)); }

/**
 * @brief Generates a range of values from the first element of the specified index tuple to the second element with a specified step size.
 * @param bes The index tuple specifying the begin and end values of the range, and the step size.
 * @return A range of values from the first element of the index tuple to the second element with a step size of the third element.
 */
AX_FORCE_INLINE auto iota(Index_tuple<3> bes) {
  return views::iota(std::get<0>(bes), std::get<1>(bes)) | views::stride(std::get<2>(bes));
}

/****************************** aliases ******************************/

using views::cartesian_product;
using views::enumerate;
using views::zip;

/****************************** multi-iota ******************************/

/**
 * @brief Generates a Cartesian product of multiple ranges of values.
 * @param args The ranges of values.
 * @return A Cartesian product of the specified ranges.
 */
template <typename... T> AX_FORCE_INLINE auto multi_iota(T &&...args) {
  return cartesian_product(iota(std::forward<T>(args))...);
}

}  // namespace ax::utils
