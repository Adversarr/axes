#pragma once

#include <ax/math/common.hpp>
namespace ax::gl {

struct Points {
  math::field3r vertices_;
  math::field4r colors_;

  real point_size_{1.0};
  bool use_global_model_{true};
};

}  // namespace ax::gl
