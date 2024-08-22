#pragma once
#include "ax/math/views.hpp"
#include "ax/utils/ndrange.hpp"
namespace ax::gl::prim {

class Grid {
public:
  Grid() {
    x_range_ = math::RealVector2{-100, 100};
    z_range_ = math::RealVector2{-100, 100};
    resolution_ = math::IndexVector2{20, 20};
  }

  Lines Draw() const {
    Index num_vertices = (resolution_[0] + 1) * (resolution_[1] + 1);
    Index num_lines = 2 * resolution_[0] * resolution_[1] + resolution_[0] + resolution_[1];
    Lines lines;
    lines.colors_ = math::make_real_field<4>(num_vertices);
    lines.colors_.setOnes();
    lines.vertices_.resize(3, num_vertices);
    lines.indices_.resize(2, num_lines);
    auto shape = math::make_shape(resolution_[0] + 1, resolution_[1] + 1);
    auto vert = math::make_accessor(lines.vertices_, shape);
    for (auto [ij, val] : math::enumerate(vert)) {
      auto [i, j] = ij;
      val[0] = x_range_[0] + i * (x_range_[1] - x_range_[0]) / resolution_[0];
      val[1] = 0;
      val[2] = z_range_[0] + j * (z_range_[1] - z_range_[0]) / resolution_[1];
    }

    Index cnt = 0;
    for (auto [i, j] : shape) {
      const Index self = shape(i, j), right = shape(i + 1, j), up = shape(i, j + 1);
      if (i < resolution_[0]) {
        lines.indices_.col(cnt++) = math::IndexVector2{self, right};
      }
      if (j < resolution_[1]) {
        lines.indices_.col(cnt++) = math::IndexVector2{self, up};
      }
    }
    return lines;
  }

private:
  math::RealVector2 x_range_;
  math::RealVector2 z_range_;
  math::IndexVector2 resolution_;
};

}  // namespace ax::gl::prim