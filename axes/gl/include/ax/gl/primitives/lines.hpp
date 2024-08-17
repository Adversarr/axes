#pragma once
#include "ax/geometry/topology.hpp"
#include "ax/gl/primitives/mesh.hpp"
#include "ax/math/common.hpp"
namespace ax::gl {

class Lines {
public:
  math::RealField3 vertices_;
  math::RealField4 colors_;
  math::IndexField2 indices_;

  math::RealField3 instance_offset_;
  math::RealField4 instance_color_;

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

  return lines;
}

}  // namespace ax::gl
