#pragma once
#include "ax/math/common.hpp"

namespace ax::gl {

struct Quiver {
  math::field3r positions_;
  math::field3r directions_;
  math::field4r colors_;

  real scale_{1.0};
  real head_ratio_{0.2};
  bool normalize_{true};
  bool use_global_model_{true};
};

}