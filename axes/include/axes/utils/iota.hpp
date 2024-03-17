#pragma once
#include <range/v3/view.hpp>  // IWYU pragma: export

#include "axes/core/common.hpp"
#include "axes/utils/dup_tuple.hpp"  // IWYU pragma: export
#include "ranges.hpp"                // IWYU pragma: export

namespace ax::utils {

/****************************** iota ******************************/

AX_FORCE_INLINE auto iota(idx end) { return views::iota(static_cast<idx>(0), end); }

AX_FORCE_INLINE auto iota(idx begin, idx end) { return views::iota(begin, end); }

AX_FORCE_INLINE auto iota(idx begin, idx end, idx step) {
  return views::iota(begin, end) | views::stride(step);
}

AX_FORCE_INLINE auto iota(idx_tuple<2> be) { return views::iota(std::get<0>(be), std::get<1>(be)); }
AX_FORCE_INLINE auto iota(idx_tuple<3> bes) {
  return views::iota(std::get<0>(bes), std::get<1>(bes)) | views::stride(std::get<2>(bes));
}

/****************************** aliases ******************************/
using views::cartesian_product;
using views::enumerate;
using views::zip;

/****************************** multi-iota ******************************/

template <typename... T> AX_FORCE_INLINE auto multi_iota(T &&...args) {
  return cartesian_product(iota(std::forward<T>(args))...);
}

}  // namespace ax::utils
