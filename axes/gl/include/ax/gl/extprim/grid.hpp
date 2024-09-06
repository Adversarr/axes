#pragma once
#include "ax/gl/primitives/lines.hpp"

namespace ax::gl::prim {

class Grid {
public:
  explicit Grid(Index res_x = 20, Index res_y = 20);

  Lines Draw() const;

  math::RealVector2 x_range_;
  math::RealVector2 z_range_;
  math::IndexVector2 resolution_;
};

}  // namespace ax::gl::prim