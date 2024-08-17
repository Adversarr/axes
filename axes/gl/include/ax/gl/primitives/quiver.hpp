#pragma once
#include "ax/math/common.hpp"

namespace ax::gl {

struct Quiver {
  math::RealField3 positions_;
  math::RealField3 directions_;
  math::RealField4 colors_;

  real scale_{1.0};
  real head_ratio_{0.2};
  bool normalize_{true};
  bool use_global_model_{true};
};

}