#include "ax/geometry/normal.hpp"

#include <igl/per_vertex_normals.h>

#include "ax/core/logging.hpp"
#include "ax/geometry/common.hpp"
#include "ax/math/linalg.hpp"
#include "ax/math/utils/structure_binding.hpp"


namespace ax::geo {
using namespace math;

RealField3 normal_per_face(RealField3 const& vertices, IndexField3 const& indices) {
  RealField3 normals(3, indices.cols());
  for (int i = 0; i < indices.cols(); ++i) {
    Index ii = indices(0, i);
    Index ij = indices(1, i);
    Index ik = indices(2, i);
    AX_DCHECK(ii < vertices.cols() && ij < vertices.cols() && ik < vertices.cols(),
              "Index out of range.", ii, ij, ik, vertices.cols());
    RealVector3 const& a = vertices.col(ii);
    RealVector3 const& b = vertices.col(ij);
    RealVector3 const& c = vertices.col(ik);
    RealVector3 const ab = b - a;
    RealVector3 const ac = c - a;
    RealVector3 const n = ab.cross(ac);
    normals.col(i) = normalized(n);
  }
  return normals;
}

RealField3 normal_per_vertex(RealField3 const& vertices, IndexField3 const& indices,
                                details::face_uniform_avg_t) {
  RealField3 normals = normal_per_face(vertices, indices);
  RealField3 vertex_normals = math::zeros<3>(vertices.cols());

  for (Index i = 0; i < indices.cols(); ++i) {
    vertex_normals.col(indices(0, i)) += normals.col(i);
    vertex_normals.col(indices(1, i)) += normals.col(i);
    vertex_normals.col(indices(2, i)) += normals.col(i);
  }

  for (Index i = 0; i < vertex_normals.cols(); ++i) {
    vertex_normals.col(i) = normalized(vertex_normals.col(i));
  }

  return vertex_normals;
}

RealField3 normal_per_vertex(RealField3 const& vertices, IndexField3 const& indices,
                                details::face_area_avg_t) {
  RealField3 vertex_normals = math::zeros<3>(vertices.cols());
  RealField1 vertex_areas = math::zeros<1>(vertices.cols());

  for (int i = 0; i < indices.cols(); ++i) {
    RealVector3 const& a = vertices.col(indices(0, i));
    RealVector3 const& b = vertices.col(indices(1, i));
    RealVector3 const& c = vertices.col(indices(2, i));
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

RealField3 normal_per_vertex(RealField3 const& vertices, IndexField3 const& indices,
                                details::face_angle_avg_t) {
  RealField3 normals = normal_per_face(vertices, indices);
  RealField3 vertex_normals = math::zeros<3>(vertices.cols());
  RealField1 vertex_angle_sum = math::zeros<1>(vertices.cols());

  for (int i = 0; i < indices.cols(); ++i) {
    RealVector3 const& a = vertices.col(indices(0, i));
    RealVector3 const& b = vertices.col(indices(1, i));
    RealVector3 const& c = vertices.col(indices(2, i));
    Triangle3 const triangle(a, b, c);

    RealVector3 const normal = triangle.Normal().normalized();

    for (int j = 0; j < 3; ++j) {
      Real angle = triangle.Angle(j);
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
