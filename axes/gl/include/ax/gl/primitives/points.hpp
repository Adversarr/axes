#pragma once

#include <ax/math/common.hpp>
namespace ax::gl {

struct Points {
  math::RealField3 vertices_;
  math::RealField4 colors_;

  Real point_size_{1.0};
  bool use_global_model_{true};
};

}  // namespace ax::gl
