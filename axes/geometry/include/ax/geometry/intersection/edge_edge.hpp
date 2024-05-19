#pragma once
#include "ax/geometry/intersection/common.hpp"
#include "ax/math/linalg.hpp"
namespace ax::geo {

// Discrete time version
AX_HOST_DEVICE AX_FORCE_INLINE CollisionInfo detect_edge_edge(Segment const& a, Segment const& b,
                                                              real tol, real dt = 0.0) {
  math::vec3r u = a.end_ - a.start_;
  math::vec3r v = b.end_ - b.start_;
  math::vec3r w = a.start_ - b.start_;
  math::vec3r n = math::cross(u, v).normalized();
  real d = math::dot(n, w);
  if (math::abs(d) < tol) {
    return CollisionInfo::EdgeEdge(a.id_, b.id_, dt);
  } else {
    return CollisionInfo();
  }
}

// Continuous time version: currently not available.
AX_HOST_DEVICE AX_FORCE_INLINE CollisionInfo detect_edge_edge(Segment const& a0, Segment const& a1,
                                                              Segment const& b0, Segment const& b1,
                                                              real tol) {
  if (auto collliding = detect_edge_edge(a0, b0, tol, 0.0)) {
    return collliding;
  } else {
    return detect_edge_edge(a1, b1, tol, 1.0);
  }
}
}