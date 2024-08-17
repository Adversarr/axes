#pragma once
#include "ax/math/common.hpp"
namespace ax::geo {

math::IndexField2 get_edges(math::IndexField3 const& triangles);
math::IndexField2 get_edges(math::IndexField4 const& tetrahedrons);

math::IndexField2 get_boundary_edges(math::IndexField3 const& triangles);

math::IndexField2 get_boundary_edges(math::RealField3 const& vertices, math::IndexField4 const& tetrahedrons);

math::IndexField3 get_boundary_triangles(math::RealField3 const& vertices, math::IndexField4 const& tetrahedrons);

}  // namespace ax::geo
