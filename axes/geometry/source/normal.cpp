#include "ax/geometry/normal.hpp"

#include <igl/per_vertex_normals.h>

#include "ax/core/logging.hpp"
#include "ax/geometry/common.hpp"
#include "ax/math/linalg.hpp"
#include "ax/math/structure_binding.hpp"


namespace ax::geo {
using namespace math;

field3r normal_per_face(field3r const& vertices, field3i const& indices) {
  field3r normals(3, indices.cols());
  for (int i = 0; i < indices.cols(); ++i) {
    idx ii = indices(0, i);
    idx ij = indices(1, i);
    idx ik = indices(2, i);
    AX_DCHECK(ii < vertices.cols() && ij < vertices.cols() && ik < vertices.cols(),
              "Index out of range.", ii, ij, ik, vertices.cols());
    vec3r const& a = vertices.col(ii);
    vec3r const& b = vertices.col(ij);
    vec3r const& c = vertices.col(ik);
    vec3r const ab = b - a;
    vec3r const ac = c - a;
    vec3r const n = ab.cross(ac);
    normals.col(i) = normalized(n);
  }
  return normals;
}

field3r normal_per_vertex(field3r const& vertices, field3i const& indices,
                                details::face_uniform_avg_t) {
  field3r normals = normal_per_face(vertices, indices);
  field3r vertex_normals = math::zeros<3>(vertices.cols());

  for (idx i = 0; i < indices.cols(); ++i) {
    auto [x, y, z] = unpack(indices.col(i).eval());

    vertex_normals.col(indices(0, i)) += normals.col(i);
    vertex_normals.col(indices(1, i)) += normals.col(i);
    vertex_normals.col(indices(2, i)) += normals.col(i);
  }

  for (idx i = 0; i < vertex_normals.cols(); ++i) {
    vertex_normals.col(i) = normalized(vertex_normals.col(i));
  }

  return vertex_normals;
}

field3r normal_per_vertex(field3r const& vertices, field3i const& indices,
                                details::face_area_avg_t) {
  field3r vertex_normals = math::zeros<3>(vertices.cols());
  field1r vertex_areas = math::zeros<1>(vertices.cols());

  for (int i = 0; i < indices.cols(); ++i) {
    vec3r const& a = vertices.col(indices(0, i));
    vec3r const& b = vertices.col(indices(1, i));
    vec3r const& c = vertices.col(indices(2, i));
    Triangle3 const triangle(a, b, c);
    vertex_normals.col(indices(0, i)) += triangle.Normal();
    vertex_normals.col(indices(1, i)) += triangle.Normal();
    vertex_normals.col(indices(2, i)) += triangle.Normal();
  }

  for (int i = 0; i < vertex_normals.cols(); ++i) {
    vertex_normals.col(i) = normalized(vertex_normals.col(i));
  }

  return vertex_normals;
}

field3r normal_per_vertex(field3r const& vertices, field3i const& indices,
                                details::face_angle_avg_t) {
  field3r normals = normal_per_face(vertices, indices);
  field3r vertex_normals = math::zeros<3>(vertices.cols());
  field1r vertex_angle_sum = math::zeros<1>(vertices.cols());

  for (int i = 0; i < indices.cols(); ++i) {
    vec3r const& a = vertices.col(indices(0, i));
    vec3r const& b = vertices.col(indices(1, i));
    vec3r const& c = vertices.col(indices(2, i));
    Triangle3 const triangle(a, b, c);

    vec3r const normal = triangle.Normal().normalized();

    for (int j = 0; j < 3; ++j) {
      real angle = triangle.Angle(j);
      vertex_normals.col(indices(j, i)) += normal * angle;
      vertex_angle_sum(indices(j, i)) += angle;
    }
  }

  for (int i = 0; i < vertex_normals.cols(); ++i) {
    vertex_normals.col(i) /= vertex_angle_sum(i);
    vertex_normals.col(i) = normalized(vertex_normals.col(i));
  }

  return vertex_normals;
}

}  // namespace ax::geo
