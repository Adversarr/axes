#pragma once

#include "ax/math/common.hpp"

namespace ax::geo {

math::RealMatrix4 look_at(math::RealVector3 const& eye, math::RealVector3 const& center, math::RealVector3 const& up);

math::RealMatrix4 perspective(Real fov, Real aspect, Real near, Real far);

math::RealMatrix4 ortho(Real left, Real right, Real bottom, Real top, Real near, Real far);

math::RealMatrix4 rotate_x(Real angle_rad);

math::RealMatrix4 rotate_y(Real angle_rad);

math::RealMatrix4 rotate_z(Real angle_rad);

math::RealMatrix4 translate(math::RealVector3 const& v);

}  // namespace ax::geo
