#pragma once

#include "axes/math/common.hpp"

namespace ax::geo {

math::mat4r look_at(math::vec3r const& eye, math::vec3r const& center, math::vec3r const& up);

math::mat4r perspective(real fov, real aspect, real near, real far);

math::mat4r ortho(real left, real right, real bottom, real top, real near, real far);

}  // namespace ax::geo
