#pragma once

#include "ax/math/common.hpp"

namespace ax::geo {

math::mat4r look_at(math::vec3r const& eye, math::vec3r const& center, math::vec3r const& up);

math::mat4r perspective(real fov, real aspect, real near, real far);

math::mat4r ortho(real left, real right, real bottom, real top, real near, real far);

math::mat4r rotate_x(real angle_rad);

math::mat4r rotate_y(real angle_rad);

math::mat4r rotate_z(real angle_rad);

math::mat4r translate(math::vec3r const& v);

}  // namespace ax::geo
