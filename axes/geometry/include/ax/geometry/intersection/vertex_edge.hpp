#pragma once
#include "ax/geometry/hittables.hpp"
#include "ax/math/polynomial.hpp"

namespace ax::geo {

AX_HOST_DEVICE AX_FORCE_INLINE CollisionInfo detect_vertex_edge(Vertex3 const& a, Segment3 const& b,
                                                                Real tol = math::epsilon<>,
                                                                Real t = 0) {
  math::RealVector3 const u = b.Direction();
  math::RealVector3 const v = a.Position() - b.Origin();
  Real const sqr_norm_u = u.squaredNorm();
  Real const norm_u = math::sqrt(sqr_norm_u);
  Real const d = math::norm(math::cross(u, v)) / norm_u;
  if (d < tol) {
    // on the line. still need to check is within the segment.
    Real const tl = math::dot(v, u);
    if (tl >= 0 && tl <= sqr_norm_u) {
      return CollisionInfo::VertexEdge(t);
    }
  }
  return CollisionInfo();
}

AX_HOST_DEVICE AX_FORCE_INLINE CollisionInfo detect_vertex_edge(Vertex3 const& a0,
                                                                Vertex3 const& a1,
                                                                Segment3 const& b0,
                                                                Segment3 const& b1, Real tol) {
  Real b = a0.Position().y(), c = a0.Position().z();
  Real B = a1.Position().y(), C = a1.Position().z();
  Real e = b0.Origin().y(), f = b0.Origin().z();
  Real E = b1.Origin().y(), F = b1.Origin().z();
  Real h = b0.End().y(), i = b0.End().z();
  Real H = b1.End().y(), I = b1.End().z();

  // quadratic eq.
  // 0 (E - b)*(c - f) - (F - c)*(b - e)
  // 1 t*((E - b)*(C - c + f - i) - (F - c)*(B - b + e - h) + (b - e)*(C + F - I - c) - (c - f)*(B +
  // E - H - b)) 2 t**2*(-(B + E - H - b)*(C - c + f - i) + (B - b + e - h)*(C + F - I - c))
  Real k0 = (E - b) * (c - f) - (F - c) * (b - e);
  Real k1 = (E - b) * (C - c + f - i) - (F - c) * (B - b + e - h) + (b - e) * (C + F - I - c)
            - (c - f) * (B + E - H - b);
  Real k2 = -(B + E - H - b) * (C - c + f - i) + (B - b + e - h) * (C + F - I - c);

  auto info = math::solve_quadratic(k2, k1, k0, 0, 1, tol);
  for (Index i = 0; i < 1; ++i)
    if (info.valid_[i]) {
      Vertex3 a{math::lerp(a0.Position(), a1.Position(), info.root_[i])};
      Segment3 b{math::lerp(b0.Origin(), b1.Origin(), info.root_[i]),
                 math::lerp(b1.Direction(), b1.Direction(), info.root_[i])};
      if (auto is_colliding = detect_vertex_edge(a, b, tol, info.root_[i])) {
        return is_colliding;
      }
    }
  return CollisionInfo();
}
}  // namespace ax::geo
