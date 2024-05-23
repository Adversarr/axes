#pragma once
#include "ax/geometry/hittables.hpp"
#include "ax/math/linalg.hpp"
#include "ax/math/polynomial.hpp"

namespace ax::geo {

// Discrete time version
AX_HOST_DEVICE AX_FORCE_INLINE CollisionInfo detect_vertex_face(CollidableVertex const& a,
                                                                CollidableTriangle const& b,
                                                                real tol, real dt = 0.0) {
  math::vec3r normal = math::normalized(b->Normal());
  real d = math::dot(normal, a->Position() - b->A());
  if (math::abs(d) < tol) {
    // project the vertex onto the face
    math::vec3r projected = a->Position() - d * normal;
    // baricentric coordinates
    math::vec3r bary = math::barycentric(projected, b->A(), b->B(), b->C());
    if (bary[0] >= -math::epsilon<> && bary[1] >= -math::epsilon<> && bary[2] >= -math::epsilon<>) {
      return CollisionInfo::VertexFace(a.id_, b.id_, dt);
    }
  }
  return CollisionInfo();
}

// Continuous time version
AX_HOST_DEVICE AX_FORCE_INLINE CollisionInfo detect_vertex_face(CollidableVertex const& a0,
                                                                CollidableVertex const& a1,
                                                                CollidableTriangle const& b0,
                                                                CollidableTriangle const& b1,
                                                                real tol) {
  math::vec3r x10 = b0->B() - b0->A();
  math::vec3r x20 = b0->C() - b0->A();
  math::vec3r v_relative_start = a0->Position() - b0->A();
  math::vec3r y10 = b1->B() - b1->A();
  math::vec3r y20 = b1->C() - b1->A();
  math::vec3r v_relative_end = a1->Position() - b1->A();

  auto a11 = x10.x();
  auto a12 = x10.y();
  auto a13 = x10.z();
  auto b11 = x20.x();
  auto b12 = x20.y();
  auto b13 = x20.z();
  auto c11 = v_relative_start.x();
  auto c12 = v_relative_start.y();
  auto c13 = v_relative_start.z();
  auto a21 = y10.x();
  auto a22 = y10.y();
  auto a23 = y10.z();
  auto b21 = y20.x();
  auto b22 = y20.y();
  auto b23 = y20.z();
  auto c21 = v_relative_end.x();
  auto c22 = v_relative_end.y();
  auto c23 = v_relative_end.z();

  real k1 = a22 * b23 * c21 - a21 * b23 * c22 + a23 * (-(b22 * c21) + b21 * c22) - a22 * b21 * c23
            + a21 * b22 * c23;
  real k2 = a22 * b23 * c11 + b23 * (-(a21 * c12) + a12 * c21 - a11 * c22 + 3 * a21 * c22)
            + a23 * (-(b12 * c21) + b22 * (-c11 + 3 * c21) + b21 * (c12 - 3 * c22) + b11 * c22)
            + b22 * (-(a13 * c21) + a21 * (c13 - 3 * c23) + a11 * c23)
            + b21 * (a13 * c22 - a12 * c23) + a21 * (-(b13 * c22) + b12 * c23)
            + a22 * (b13 * c21 - 3 * b23 * c21 - b11 * c23 + b21 * (-c13 + 3 * c23));

  real k3 = ((-a13) * b22 + a22 * (b13 - 2 * b23) + a12 * b23) * c11
            + b23 * ((-a11 + 2 * a21) * c12 + 2 * a11 * c22 - 3 * a21 * c22)
            + b13 * ((-a21) * c12 + a12 * c21 - a11 * c22 + 2 * a21 * c22)
            + a13 * (b21 * c12 + 2 * b22 * c21 + b11 * c22 - 2 * b21 * c22)
            + a23
                  * (2 * b22 * c11 - 2 * b21 * c12 - 3 * b22 * c21 + b12 * (-c11 + 2 * c21)
                     + b11 * (c12 - 2 * c22) + 3 * b21 * c22)
            + b12 * ((-a13) * c21 + a21 * (c13 - 2 * c23) + a11 * c23)
            + a22
                  * (-2 * b13 * c21 + 3 * b23 * c21 + b21 * (2 * c13 - 3 * c23)
                     + b11 * (-c13 + 2 * c23))
            + a12 * (-2 * b23 * c21 - b11 * c23 + b21 * (-c13 + 2 * c23))
            + b22 * (a11 * (c13 - 2 * c23) + a21 * (-2 * c13 + 3 * c23));
  real k4 = (-(a22 * b13) - a23 * b22 + a12 * (b13 - b23) + a22 * b23) * c11
            + b13 * ((-a11 + a21) * c12 + a22 * c21 + a11 * c22 - a21 * c22)
            + b23 * ((a11 - a21) * c12 - a11 * c22 + a21 * c22)
            + a13
                  * (b22 * c11 - b21 * c12 - b22 * c21 + b12 * (-c11 + c21) + b11 * (c12 - c22)
                     + b21 * c22)
            + a23 * (b21 * c12 + b22 * c21 - b21 * c22 + b11 * (-c12 + c22))
            + b22 * (a21 * (c13 - c23) + a11 * (-c13 + c23))
            + b12 * (a23 * c11 - a23 * c21 + a11 * (c13 - c23) + a21 * (-c13 + c23))
            + a12 * (-(b13 * c21) + b23 * c21 + b21 * (c13 - c23) + b11 * (-c13 + c23))
            + a22 * (-(b23 * c21) + b11 * (c13 - c23) + b21 * (-c13 + c23));

  auto toi = math::solve_cubic(k4, k3, k2, k1, 0, 1, tol, 32);
  for (idx i = 0; i < 3; ++i) {
    if (!toi.valid_[i]) continue;
    real t = toi.root_[i];
    Vertex3 a{math::lerp(a0->Position(), a1->Position(), t)};
    Triangle3 b{math::lerp(b0->A(), b1->A(), t), math::lerp(b0->B(), b1->B(), t),
                math::lerp(b0->C(), b1->C(), t)};

    CollisionInfo info = detect_vertex_face({a0.Id(), a}, {b0.Id(), b}, tol, t);
    if (info) return info;
  }
  return CollisionInfo();
}
}  // namespace ax::geo
