#pragma once
#include "ax/geometry/hittables.hpp"
#include "ax/math/linalg.hpp"
#include "ax/math/polynomial.hpp"
namespace ax::geo {

// Discrete time version
AX_HOST_DEVICE AX_FORCE_INLINE CollisionInfo detect_edge_edge(Segment3 const& a, Segment3 const& b,
                                                              Real tol, Real t = 0.0) {
  math::RealVector3 u = math::normalized(a.Direction());
  math::RealVector3 v = math::normalized(b.Direction());
  Real const distance_normal = math::cross(u, b.Origin() - a.Origin()).norm();

  if (math::cross(u, v).norm() < 1e-8) {
    if (distance_normal > tol) {
      return CollisionInfo();
    }
    // parallel. need to test if they are on the same line.
    Real const b_origin = math::dot(u, b.Origin() - a.Origin());
    Real const b_end = math::dot(u, b.End() - a.Origin());
    Real const a_end = math::dot(u, a.Direction());
    auto inrange = [a_end, tol](Real x) -> bool {
      if (a_end < 0) {
        return -a_end - tol < x && x < tol;
      } else {
        return -tol < x && x < a_end + tol;
      }
    };

    // std::cout << b_origin << " " << b_end << " " << a_end << std::endl;
    if (inrange(b_origin)) {
      return CollisionInfo::EdgeEdge(t);
    } else if (inrange(b_end)) {
      return CollisionInfo::EdgeEdge(t);
    } else {
      return CollisionInfo();
    }
  }
  // math::RealVector3 A = a.Origin(), B = a.End(), C = b.Origin(), D = b.End();
  // if (u.dot(v) < 0) {
  //   std::swap(A, B);
  //   u.noalias() = -u;
  // }

  // math::RealVector3 abc = math::cross(B - A, C - A);
  // math::RealVector3 bad = math::cross(B - A, D - A);
  // math::RealVector3 cbd = math::cross(D - B, C - B);

  // Real const t1 = (abc + bad).norm();
  // Real const t2 = (-abc + cbd).norm();
  // std::cout << t1 << " " << t2 << std::endl;
  // if (abs(t1 - t2) < tol) {
  //   return CollisionInfo::EdgeEdge(a.id_, b.id_, t);
  // }

  math::RealVector3 const R_cross = math::cross(a.Direction(), b.Direction());
  Real const distance = abs(math::dot(math::normalized(R_cross), b.Origin() - a.Origin()));
  if (distance > tol) {
    return CollisionInfo();
  }

  // std::cout << distance << std::endl;

  math::RealVector3 const ba = b.Origin() - a.Origin();
  math::RealVector3 const tt = math::cross(ba, b.Direction());
  math::RealVector3 const uu = math::cross(ba, a.Direction());

  Real const t1 = math::dot(tt, R_cross) / R_cross.squaredNorm();
  Real const t2 = math::dot(uu, R_cross) / R_cross.squaredNorm();
  if (t1 < tol || t1 > 1 - tol || t2 < tol || t2 > 1 - tol) {
    return CollisionInfo();
  }

  // std::cout << t1 << " " << t2 << std::endl;
  // validate the collision
  math::RealVector3 const p = a.Origin() + a.Direction() * t1;
  math::RealVector3 const q = b.Origin() + b.Direction() * t2;
  if ((p - q).norm() > tol) {
    return CollisionInfo();
  }

  return CollisionInfo::EdgeEdge(t);
}

// Continuous time version: currently not available.
AX_HOST_DEVICE AX_FORCE_INLINE CollisionInfo detect_edge_edge(Segment3 const& a0,
                                                              Segment3 const& a1,
                                                              Segment3 const& b0,
                                                              Segment3 const& b1, Real tol) {
  math::RealVector3 const abc = a0.Direction();
  math::RealVector3 const ABC = a1.Direction();
  math::RealVector3 const def = b0.Origin() - a0.Origin();
  math::RealVector3 const DEF = b1.Origin() - a1.Origin();
  math::RealVector3 const ghi = b0.End() - a0.Origin();
  math::RealVector3 const GHI = b1.End() - a1.Origin();
  Real const a = abc[0], b = abc[1], c = abc[2], A = ABC[0], B = ABC[1], C = ABC[2];
  Real const d = def[0], e = def[1], f = def[2], D = DEF[0], E = DEF[1], F = DEF[2];
  Real const g = ghi[0], h = ghi[1], i = ghi[2], G = GHI[0], H = GHI[1], I = GHI[2];
  Real const k4
      = (A * E * I - A * E * i - A * F * H + A * F * h + A * H * f - A * I * e + A * e * i
         - A * f * h - B * D * I + B * D * i + B * F * G - B * F * g - B * G * f + B * I * d
         - B * d * i + B * f * g + C * D * H - C * D * h - C * E * G + C * E * g + C * G * e
         - C * H * d + C * d * h - C * e * g - D * H * c + D * I * b - D * b * i + D * c * h
         + E * G * c - E * I * a + E * a * i - E * c * g - F * G * b + F * H * a - F * a * h
         + F * b * g + G * b * f - G * c * e - H * a * f + H * c * d + I * a * e - I * b * d
         - a * e * i + a * f * h + b * d * i - b * f * g - c * d * h + c * e * g);
  Real const k3
      = (A * E * i - A * F * h - A * H * f + A * I * e - 2 * A * e * i + 2 * A * f * h - B * D * i
         + B * F * g + B * G * f - B * I * d + 2 * B * d * i - 2 * B * f * g + C * D * h - C * E * g
         - C * G * e + C * H * d - 2 * C * d * h + 2 * C * e * g + D * H * c - D * I * b
         + 2 * D * b * i - 2 * D * c * h - E * G * c + E * I * a - 2 * E * a * i + 2 * E * c * g
         + F * G * b - F * H * a + 2 * F * a * h - 2 * F * b * g - 2 * G * b * f + 2 * G * c * e
         + 2 * H * a * f - 2 * H * c * d - 2 * I * a * e + 2 * I * b * d + 3 * a * e * i
         - 3 * a * f * h - 3 * b * d * i + 3 * b * f * g + 3 * c * d * h - 3 * c * e * g);
  Real const k2 = (A * e * i - A * f * h - B * d * i + B * f * g + C * d * h - C * e * g - D * b * i
                   + D * c * h + E * a * i - E * c * g - F * a * h + F * b * g + G * b * f
                   - G * c * e - H * a * f + H * c * d + I * a * e - I * b * d - 3 * a * e * i
                   + 3 * a * f * h + 3 * b * d * i - 3 * b * f * g - 3 * c * d * h + 3 * c * e * g);
  Real const k1 = a * e * i - a * f * h - b * d * i + b * f * g + c * d * h - c * e * g;
  auto toi = math::solve_cubic(k4, k3, k2, k1, 0, 1, tol * tol, 32);
  for (Index i = 0; i < 3; ++i) {
    if (!toi.valid_[i]) continue;
    Real const t = toi.root_[i];
    // std::cout << "t=" << t << std::endl;
    Segment3 a{math::lerp(a0.Origin(), a1.Origin(), t),
               math::lerp(a0.Direction(), a1.Direction(), t)};
    Segment3 b{math::lerp(b0.Origin(), b1.Origin(), t),
               math::lerp(b0.Direction(), b1.Direction(), t)};
    CollisionInfo info = detect_edge_edge(a, b, tol, t);
    if (info) return info;
  }
  return CollisionInfo();
}
}  // namespace ax::geo
