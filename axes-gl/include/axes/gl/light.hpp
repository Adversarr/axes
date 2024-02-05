#pragma once
#include "axes/math/common.hpp"

namespace ax::gl {

struct Light {
  math::vec3f position_;
  f32 ambient_strength_{0.1};
  f32 diffuse_strength_{0.5};
  f32 specular_strength_{0.4};
};

}
