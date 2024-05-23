#pragma once
#include "ax/geometry/hittables.hpp"
#include "ax/math/polynomial.hpp"

namespace ax::geo {

AX_HOST_DEVICE AX_FORCE_INLINE CollisionInfo detect_vertex_edge(CollidableVertex const& a,
                                                                CollidableSegment const& b,
                                                                real tol = math::epsilon<>,
                                                                real t = 0) {
  math::vec3r const u = b->Direction();
  math::vec3r const v = a->Position() - b->Origin();
  real const norm_u = math::norm(u);
  real const d = math::norm(math::cross(u, v)) / norm_u;
  if (d < tol) {
    // on the line. still need to check is within the segment.
    real const tl = math::dot(v, u);
    if (tl >= tol && tl <= norm_u * norm_u * (1 + tol)) {
      return CollisionInfo::VertexEdge(a.id_, b.id_, t);
    }
  }
  return CollisionInfo();
}

AX_HOST_DEVICE AX_FORCE_INLINE CollisionInfo detect_vertex_edge(CollidableVertex const& a0,
                                                                CollidableVertex const& a1,
                                                                CollidableSegment const& b0,
                                                                CollidableSegment const& b1,
                                                                real tol) {
  real b = a0->Position().y(), c = a0->Position().z();
  real B = a1->Position().y(), C = a1->Position().z();
  real e = b0->Origin().y(), f = b0->Origin().z();
  real E = b1->Origin().y(), F = b1->Origin().z();
  real h = b0->End().y(), i = b0->End().z();
  real H = b1->End().y(), I = b1->End().z();

  // quadratic eq.
  // 0 (E - b)*(c - f) - (F - c)*(b - e)
  // 1 t*((E - b)*(C - c + f - i) - (F - c)*(B - b + e - h) + (b - e)*(C + F - I - c) - (c - f)*(B +
  // E - H - b)) 2 t**2*(-(B + E - H - b)*(C - c + f - i) + (B - b + e - h)*(C + F - I - c))
  real k0 = (E - b) * (c - f) - (F - c) * (b - e);
  real k1 = (E - b) * (C - c + f - i) - (F - c) * (B - b + e - h) + (b - e) * (C + F - I - c)
            - (c - f) * (B + E - H - b);
  real k2 = -(B + E - H - b) * (C - c + f - i) + (B - b + e - h) * (C + F - I - c);

  auto info = math::solve_quadratic(k2, k1, k0, 0, 1, tol);
  for (idx i = 0; i < 1; ++i)
    if (info.valid_[i]) {
      CollidableVertex a{a0.id_,
                         Vertex3{math::lerp(a0->Position(), a1->Position(), info.root_[i])}};
      CollidableSegment b{b0.id_,
                          Segment3{math::lerp(b0->Origin(), b1->Origin(), info.root_[i]),
                                   math::lerp(b1->Direction(), b1->Direction(), info.root_[i])}};
      if (auto is_colliding = detect_vertex_edge(a, b, tol, info.root_[i])) {
        return is_colliding;
      }
    }
  return CollisionInfo();
}
}  // namespace ax::geo
