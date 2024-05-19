#pragma once
#include "ax/geometry/intersection/common.hpp"
#include "ax/math/linalg.hpp"
namespace ax::math {

AX_HOST_DEVICE AX_FORCE_INLINE math::vec3r barycentric(math::vec3r const& p, math::vec3r const& a,
                                                       math::vec3r const& b, math::vec3r const& c) {
  math::vec3r v0 = b - a, v1 = c - a, v2 = p - a;
  real d00 = math::dot(v0, v0);
  real d01 = math::dot(v0, v1);
  real d11 = math::dot(v1, v1);
  real d20 = math::dot(v2, v0);
  real d21 = math::dot(v2, v1);
  real denom = d00 * d11 - d01 * d01;
  real v = (d11 * d20 - d01 * d21) / denom;
  real w = (d00 * d21 - d01 * d20) / denom;
  real u = 1.0 - v - w;
  return math::vec3r(u, v, w);
}
}  // namespace ax::math

namespace ax::geo {

// Discrete time version
AX_HOST_DEVICE AX_FORCE_INLINE CollisionInfo detect_vertex_face(Vertex const& a, Face const& b,
                                                                real tol, real dt = 0.0) {
  math::vec3r normal = math::normalized(
      math::cross(b.vertices_[1] - b.vertices_[0], b.vertices_[2] - b.vertices_[0]));
  real d = math::dot(normal, a.position_ - b.vertices_[0]);
  if (math::abs(d) < tol) {
    // project the vertex onto the face
    math::vec3r projected = a.position_ - d * normal;
    // baricentric coordinates
    math::vec3r bary = math::barycentric(projected, b.vertices_[0], b.vertices_[1], b.vertices_[2]);
    if (bary[0] >= -tol && bary[1] >= -tol && bary[2] >= -tol) {
      return CollisionInfo::VertexFace(a.id_, b.id_, dt);
    }
  }
  return CollisionInfo();
}

// Continuous time version
AX_HOST_DEVICE AX_FORCE_INLINE CollisionInfo detect_vertex_face(Vertex const& a0, Vertex const& a1,
                                                                Face const& b0, Face const& b1,
                                                                real tol) {
  // TODO: implement continuous time vertex-face collision
  if (auto collliding = detect_vertex_face(a0, b0, tol, 0.0)) {
    return collliding;
  } else {
    return detect_vertex_face(a1, b1, tol, 1.0);
  }
}
}  // namespace ax::geo