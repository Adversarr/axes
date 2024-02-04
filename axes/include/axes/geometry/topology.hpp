#pragma once
#include "axes/math/common.hpp"
namespace ax::geo {

math::field2i get_edges(math::field3i const& triangles);

math::field2i get_boundary_edges(math::field3i const& triangles);

math::field2i get_boundary_edges(math::field4i const& tetrahedrons);

math::field3i get_boundary_triangles(math::field4i const& tetrahedrons);

// NOTE: Difficult, NotImplemented
math::field3i fix_boundary_orientation(math::field3r const& vertices, math::field4i const& tetrahedrons,
                                       math::field3i const& boundary_triangles);

}  // namespace ax::geo
