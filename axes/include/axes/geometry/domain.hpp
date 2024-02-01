#pragma once
#include "axes/math/common.hpp"

namespace ax::geo {

template <idx dim> class Domain {
public:

private:
  math::fieldr<dim> vertices_;
  math::fieldi<dim + 1> elements_;
};

}  // namespace ax::geo
