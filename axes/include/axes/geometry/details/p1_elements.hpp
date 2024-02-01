#pragma once

#include "axes/math/common.hpp"
namespace ax::geo::details {

/****************************** P1 element ******************************/
/**
 * P1 element is nothing but
 * 1. tetreaheadral element, or
 * 2. triangle element, or
 * 3. line segment element.
 */
template <idx dim> class P1ElementN {
public:
  using value_type = math::vecr<dim>;
};
}  // namespace ax::geo::details
