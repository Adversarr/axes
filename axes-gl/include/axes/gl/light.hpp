#pragma once
#include "axes/math/common.hpp"

namespace ax::gl {

struct Light {
  math::vec3r position_;
  real ambient_strength_;
};

}
