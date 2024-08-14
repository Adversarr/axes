#pragma once
#include "ax/math/common.hpp"
namespace ax::geo {

math::field2i get_edges(math::field3i const& triangles);
math::field2i get_edges(math::field4i const& tetrahedrons);

math::field2i get_boundary_edges(math::field3i const& triangles);

math::field2i get_boundary_edges(math::field3r const& vertices, math::field4i const& tetrahedrons);

math::field3i get_boundary_triangles(math::field3r const& vertices, math::field4i const& tetrahedrons);

}  // namespace ax::geo
