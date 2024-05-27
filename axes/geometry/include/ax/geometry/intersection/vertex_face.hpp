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
  math::vec3r const abc = b0->A() - a0->Position();
  math::vec3r const ABC = b1->A() - a1->Position();
  math::vec3r const def = b0->B() - a0->Position();
  math::vec3r const DEF = b1->B() - a1->Position();
  math::vec3r const ghi = b0->C() - a0->Position();
  math::vec3r const GHI = b1->C() - a1->Position();
  real const a = abc[0], b = abc[1], c = abc[2], A = ABC[0], B = ABC[1], C = ABC[2];
  real const d = def[0], e = def[1], f = def[2], D = DEF[0], E = DEF[1], F = DEF[2];
  real const g = ghi[0], h = ghi[1], i = ghi[2], G = GHI[0], H = GHI[1], I = GHI[2];
  real const k4
      = (A * E * I - A * E * i - A * F * H + A * F * h + A * H * f - A * I * e + A * e * i
         - A * f * h - B * D * I + B * D * i + B * F * G - B * F * g - B * G * f + B * I * d
         - B * d * i + B * f * g + C * D * H - C * D * h - C * E * G + C * E * g + C * G * e
         - C * H * d + C * d * h - C * e * g - D * H * c + D * I * b - D * b * i + D * c * h
         + E * G * c - E * I * a + E * a * i - E * c * g - F * G * b + F * H * a - F * a * h
         + F * b * g + G * b * f - G * c * e - H * a * f + H * c * d + I * a * e - I * b * d
         - a * e * i + a * f * h + b * d * i - b * f * g - c * d * h + c * e * g);
  real const k3
      = (A * E * i - A * F * h - A * H * f + A * I * e - 2 * A * e * i + 2 * A * f * h - B * D * i
         + B * F * g + B * G * f - B * I * d + 2 * B * d * i - 2 * B * f * g + C * D * h - C * E * g
         - C * G * e + C * H * d - 2 * C * d * h + 2 * C * e * g + D * H * c - D * I * b
         + 2 * D * b * i - 2 * D * c * h - E * G * c + E * I * a - 2 * E * a * i + 2 * E * c * g
         + F * G * b - F * H * a + 2 * F * a * h - 2 * F * b * g - 2 * G * b * f + 2 * G * c * e
         + 2 * H * a * f - 2 * H * c * d - 2 * I * a * e + 2 * I * b * d + 3 * a * e * i
         - 3 * a * f * h - 3 * b * d * i + 3 * b * f * g + 3 * c * d * h - 3 * c * e * g);
  real const k2 = (A * e * i - A * f * h - B * d * i + B * f * g + C * d * h - C * e * g - D * b * i
                   + D * c * h + E * a * i - E * c * g - F * a * h + F * b * g + G * b * f
                   - G * c * e - H * a * f + H * c * d + I * a * e - I * b * d - 3 * a * e * i
                   + 3 * a * f * h + 3 * b * d * i - 3 * b * f * g - 3 * c * d * h + 3 * c * e * g);
  real const k1 = a * e * i - a * f * h - b * d * i + b * f * g + c * d * h - c * e * g;
  auto toi = math::solve_cubic(k4, k3, k2, k1, 0, 1, tol * tol, 32);
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
