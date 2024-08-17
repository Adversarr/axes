#pragma once
#include "ax/gl/primitives/lines.hpp"
#include "ax/math/common.hpp"

namespace ax::gl::prim {

class Axes {
public:
  inline Axes()
      : position_(math::zeros<3>()),
        scale_(math::constant<3>(real(5))),
        color_(math::eye<3>()),
        rotate_(math::eye<3>()) {}
  math::RealVector3 position_;
  math::RealVector3 scale_;
  math::RealMatrix3 color_;
  math::RealMatrix3 rotate_;

  Lines Draw() const {
    Lines lines;
    lines.vertices_ = math::zeros<3>(6);
    lines.vertices_.col(0) = position_;
    lines.vertices_.col(1) = position_ + rotate_ * scale_.x() * math::RealVector3::UnitX();
    lines.vertices_.col(2) = position_;
    lines.vertices_.col(3) = position_ + rotate_ * scale_.y() * math::RealVector3::UnitY();
    lines.vertices_.col(4) = position_;
    lines.vertices_.col(5) = position_ + rotate_ * scale_.z() * math::RealVector3::UnitZ();
    lines.colors_ = math::zeros<4>(6);
    lines.colors_.block<3, 1>(0, 0) = color_.col(0);
    lines.colors_.block<3, 1>(0, 1) = color_.col(0);
    lines.colors_.block<3, 1>(0, 2) = color_.col(1);
    lines.colors_.block<3, 1>(0, 3) = color_.col(1);
    lines.colors_.block<3, 1>(0, 4) = color_.col(2);
    lines.colors_.block<3, 1>(0, 5) = color_.col(2);
    lines.indices_ = math::zeros<2, Index>(3);
    lines.indices_.col(0) = math::IndexVec2{0, 1};
    lines.indices_.col(1) = math::IndexVec2{2, 3};
    lines.indices_.col(2) = math::IndexVec2{4, 5};
    lines;
    return lines;
  }
};
}  // namespace ax::gl::prim
