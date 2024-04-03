#pragma once
#include "ax/math/common.hpp"

namespace ax::gl {

struct Light {
  math::vec3f position_;
  f32 ambient_strength_{0.1f};
  f32 diffuse_strength_{0.5f};
  f32 specular_strength_{0.4f};
};

}
