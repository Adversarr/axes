#pragma once
#include <axes/math/common.hpp>

namespace ax::gui {

class Mesh3D {
public:
  math::field3r vertices_;
  math::field3r colors_;
  math::field3r normals_;
  math::field3i indices_;

  math::field3r instance_offset_;
  math::field3r instance_color_;

  Mesh3D() = default;

  Mesh3D& SetVertices(math::field3r vertices);
  Mesh3D& SetColors(math::field3r colors);
  Mesh3D& SetNormals(math::field3r normals);
  Mesh3D& SetIndices(math::field3i indices);

  void Flush();
  void FlushVerticesOnly();

  bool flush_{false};
  bool flush_vo_{false};
};

}  // namespace ax::gui
