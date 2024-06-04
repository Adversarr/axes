#pragma once
#include "ax/geometry/hittables.hpp"
#include "ax/math/polynomial.hpp"

namespace ax::geo {

AX_HOST_DEVICE AX_FORCE_INLINE CollisionInfo detect_vertex_vertex(Vertex3 const& a,
                                                                  Vertex3 const& b,
                                                                  real tol = math::epsilon<>,
                                                                  real t = 0) {
  math::vec3r const u = a.Position() - b.Position();
  real const d = math::norm(u);
  if (d < tol) {
    return CollisionInfo::VertexVertex(t);
  } else {
    return CollisionInfo();
  }
}

AX_HOST_DEVICE AX_FORCE_INLINE CollisionInfo detect_vertex_vertex(Vertex3 const& a0,
                                                                  Vertex3 const& a1,
                                                                  Vertex3 const& b0,
                                                                  Vertex3 const& b1, real tol) {
  // fallback to discrete version
  math::vec3r const u = a0.Position() - a1.Position();  // a,b,c
  math::vec3r const v = b0.Position() - b1.Position();  // d,e,f
  // a * a + b * b + c * c
  real const k0 = u.dot(u);
  // a(−2a+2d)+b(−2b+2e)+c(−2c+2f))
  real const k1 = 2 * u.dot(v) - 2 * u.dot(u);
  // (a−d) 2 +(b−e) 2 +(c−f) 2
  real const k2 = (u - v).dot(u - v);
  auto quad = math::solve_quadratic(k0, k1, k2, 0, 1, tol);
  for (idx i = 0; i < 1; ++i) {
    if (quad.valid_[i]) {
      real t = quad.root_[i];
      Vertex3 a(a0.Position() + t * (a1.Position() - a0.Position()));
      Vertex3 b(b0.Position() + t * (b1.Position() - b0.Position()));
      if (auto t0 = detect_vertex_vertex(a, b, tol, t)) {
        return t0;
      }
    }
  }
  return CollisionInfo();
}

}  // namespace ax::geo
