#pragma once
#include <ax/math/common.hpp>

namespace ax::gl {

struct Mesh {
  math::RealField3 vertices_;
  math::RealField4 colors_;
  math::RealField3 normals_;
  math::IndexField3 indices_;

  math::RealField3 instance_offset_;
  math::RealField3 instance_scale_;
  math::RealField4 instance_color_;

  bool is_flat_{false};
  bool use_lighting_{true};
  bool use_global_model_{true};
};

}  // namespace ax::gl
