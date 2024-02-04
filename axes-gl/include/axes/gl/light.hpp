#pragma once
#include "axes/math/common.hpp"

namespace ax::gl {

struct Light {
  math::vec3f position_;
  f32 ambient_strength_;
};

}
