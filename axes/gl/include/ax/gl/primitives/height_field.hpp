#pragma once

#include "ax/gl/primitives/mesh.hpp"

namespace ax::gl {

StatusOr<Mesh> make_height_field(math::vecxr const& z, idx nx, idx ny);

}  // namespace ax::gl