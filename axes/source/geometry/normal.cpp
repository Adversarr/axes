#include "axes/geometry/normal.hpp"

#include "axes/math/linalg.hpp"

namespace ax::geo {

math::field3r normal_per_face(math::field3r const& vertices, math::field3i const& indices) {
  math::field3r normals(3, indices.cols());
  for (int i = 0; i < indices.cols(); ++i) {
    math::vec3r const& a = vertices.col(indices(0, i));
    math::vec3r const& b = vertices.col(indices(1, i));
    math::vec3r const& c = vertices.col(indices(2, i));
    math::vec3r const ab = b - a;
    math::vec3r const ac = c - a;
    math::vec3r const n = ab.cross(ac);
    normals.col(i) = math::normalized(n);
  }
  return normals;
}

math::field3r normal_per_vertex(math::field3r const& vertices, math::field3i const& indices,
                                details::face_uniform_avg_t) {
  math::field3r normals = normal_per_face(vertices, indices);
  math::field3r vertex_normals = math::zeros<3>(vertices.cols());

  for (int i = 0; i < indices.cols(); ++i) {
    vertex_normals.col(indices(0, i)) += normals.col(i);
    vertex_normals.col(indices(1, i)) += normals.col(i);
    vertex_normals.col(indices(2, i)) += normals.col(i);
  }

  for (int i = 0; i < vertex_normals.cols(); ++i) {
    vertex_normals.col(i) = math::normalized(vertex_normals.col(i));
  }

  return vertex_normals;
}

}  // namespace ax::geo
