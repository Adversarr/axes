#pragma once
#include <ax/math/common.hpp>

namespace ax::gl {

struct Mesh {
  math::field3r vertices_;
  math::field4r colors_;
  math::field3r normals_;
  math::field3i indices_;

  math::field3r instance_offset_;
  math::field3r instance_scale_;
  math::field4r instance_color_;

  bool is_flat_{false};
  bool use_lighting_{true};
  bool use_global_model_{true};
};

}  // namespace ax::gl
