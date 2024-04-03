#pragma once
#include "ax/geometry/topology.hpp"
#include "ax/gl/primitives/mesh.hpp"
#include "ax/math/common.hpp"
namespace ax::gl {

class Lines {
public:
  math::field3r vertices_;
  math::field4r colors_;
  math::field2i indices_;

  math::field3r instance_offset_;
  math::field4r instance_color_;

  bool flush_{true};
  bool use_global_model_{true};

  static Lines Create(Mesh const& mesh);
};

inline Lines Lines::Create(Mesh const& mesh) {
  Lines lines;
  lines.vertices_ = mesh.vertices_;
  lines.colors_ = mesh.colors_;
  lines.indices_ = geo::get_edges(mesh.indices_);

  lines.instance_offset_ = mesh.instance_offset_;
  lines.instance_color_ = mesh.instance_color_;

  lines.flush_ = mesh.flush_;
  return lines;
}

}  // namespace ax::gl
