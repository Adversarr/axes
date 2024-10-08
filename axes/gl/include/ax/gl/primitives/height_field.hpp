#pragma once

#include "ax/gl/primitives/mesh.hpp"

namespace ax::gl {

Mesh make_height_field(math::RealVectorX const& z, Index nx, Index ny);

}  // namespace ax::gl
