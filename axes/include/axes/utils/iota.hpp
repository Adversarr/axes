#pragma once
#include "axes/core/common.hpp"
#include "ranges.hpp"  // IWYU pragma: export

namespace ax::utils {

/****************************** iota ******************************/

AXES_FORCE_INLINE auto iota(idx end) { return views::iota(static_cast<idx>(0), end); }

AXES_FORCE_INLINE auto iota(idx begin, idx end) { return views::iota(begin, end); }

}  // namespace ax::utils
