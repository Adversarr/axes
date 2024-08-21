#pragma once
#include <range/v3/range_fwd.hpp>  // IWYU pragma: export

/**
 * NOTE: Export all ranges and views from the ranges-v3 library.
 *       If STL ranges are ever added to the standard, this file
 *       will need to be updated to export those as well.
 */

namespace ax::utils {

/****************************** Ranges ******************************/
namespace ranges {
using namespace ::ranges;
}

/****************************** Views ******************************/

namespace views {
using namespace ::ranges::views;
}
}  // namespace ax::utils
