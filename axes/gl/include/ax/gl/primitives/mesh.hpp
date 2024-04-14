#pragma once
#include <ax/math/common.hpp>

namespace ax::gl {

class Mesh {
public:
  math::field3r vertices_;
  math::field4r colors_;
  math::field3r normals_;
  math::field3i indices_;

  math::field3r instance_offset_;
  math::field4r instance_color_;

  void Flush();
  void FlushVerticesOnly();

  bool flush_{true};
  bool is_flat_{false};
  bool use_lighting_{true};
  bool use_global_model_{true};
};

}  // namespace ax::gl
