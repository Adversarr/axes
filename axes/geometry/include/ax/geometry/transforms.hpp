#pragma once

#include "ax/math/common.hpp"

namespace ax::geo {

math::RealMatrix4 look_at(math::RealVector3 const& eye, math::RealVector3 const& center, math::RealVector3 const& up);

math::RealMatrix4 perspective(real fov, real aspect, real near, real far);

math::RealMatrix4 ortho(real left, real right, real bottom, real top, real near, real far);

math::RealMatrix4 rotate_x(real angle_rad);

math::RealMatrix4 rotate_y(real angle_rad);

math::RealMatrix4 rotate_z(real angle_rad);

math::RealMatrix4 translate(math::RealVector3 const& v);

}  // namespace ax::geo
