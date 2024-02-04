#pragma once
#include <axes/math/common.hpp>

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

  bool flush_{false};
  bool is_flat_{false};
  bool use_lighting_{false};
};

}  // namespace ax::gl
