#pragma once
#include "ax/geometry/hittables.hpp"
#include "ax/math/linalg.hpp"
#include "ax/math/polynomial.hpp"
namespace ax::geo {

// Discrete time version
AX_HOST_DEVICE AX_FORCE_INLINE CollisionInfo detect_edge_edge(CollidableSegment const& a,
                                                              CollidableSegment const& b, real tol,
                                                              real t = 0.0) {
  auto const& u = a->Direction();
  auto const& v = b->Direction();
  math::vec3r const w = a->Origin() - b->Origin();
  math::vec3r const n = math::cross(u, v).normalized();
  real d = math::dot(n, w);

  if (math::abs(d) < tol) {
    return CollisionInfo::EdgeEdge(a.id_, b.id_, t);
  } else {
    return CollisionInfo();
  }
}

// Continuous time version: currently not available.
AX_HOST_DEVICE AX_FORCE_INLINE CollisionInfo detect_edge_edge(CollidableSegment const& a0,
                                                              CollidableSegment const& a1,
                                                              CollidableSegment const& b0,
                                                              CollidableSegment const& b1,
                                                              real tol) {
  // Vec3<real> e0_v1rs = edge0_vertex1_start - edge0_vertex0_start;
  // Vec3<real> e1_v0rs = edge1_vertex0_start - edge0_vertex0_start;
  // Vec3<real> e1_v1rs = edge1_vertex1_start - edge0_vertex0_start;
  // Vec3<real> e0_v1re = edge0_vertex1_end - edge0_vertex0_end;
  // Vec3<real> e1_v0re = edge1_vertex0_end - edge0_vertex0_end;
  // Vec3<real> e1_v1re = edge1_vertex1_end - edge0_vertex0_end;

  auto e0_v1rs = a0->Direction();
  auto e1_v0rs = b0->Origin() - a0->Origin();
  auto e1_v1rs = b0->End() - a0->Origin();
  auto e0_v1re = a1->Direction();
  auto e1_v0re = b1->Origin() - a1->Origin();
  auto e1_v1re = b1->End() - a1->Origin();

  auto a11 = e1_v0rs.x();
  auto a12 = e1_v0rs.y();
  auto a13 = e1_v0rs.z();
  auto b11 = e1_v1rs.x();
  auto b12 = e1_v1rs.y();
  auto b13 = e1_v1rs.z();
  auto c11 = e0_v1rs.x();
  auto c12 = e0_v1rs.y();
  auto c13 = e0_v1rs.z();
  auto a21 = e1_v0re.x();
  auto a22 = e1_v0re.y();
  auto a23 = e1_v0re.z();
  auto b21 = e1_v1re.x();
  auto b22 = e1_v1re.y();
  auto b23 = e1_v1re.z();
  auto c21 = e0_v1re.x();
  auto c22 = e0_v1re.y();
  auto c23 = e0_v1re.z();

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
    CollidableSegment a{a0.id_, Segment3{math::lerp(a0->Origin(), a1->Origin(), t),
                                         math::lerp(a0->End(), a1->End(), t)}};
    CollidableSegment b{b0.id_, Segment3{math::lerp(b0->Origin(), b1->Origin(), t),
                                         math::lerp(b0->End(), b1->End(), t)}};
    CollisionInfo info = detect_edge_edge(a, b, tol, t);
    if (info) return info;
  }
  return CollisionInfo();
}
}  // namespace ax::geo
