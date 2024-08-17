#pragma once
#include "ax/math/linalg.hpp"

namespace ax::xpbd {

using v3 = math::RealVector3;
using m32 = math::RealMatrix<3, 2>;
using m3 = math::RealMatrix<3, 3>;
using m34 = math::RealMatrix<3, 4>;
using m4 = math::RealMatrix<4, 4>;

inline bool relax_vertex_triangle_impl(m34 const& z, m34 const& u, m34 const& o, m34& x, real rho,
                                       real& k, real tol) {
  // k/2(|| Dx || - d)^2 + rho/2 ||x-(z-u)||^2
  // What i wish: x - u no collision => instead of solving x, solve x - u ?
  // first, test if there is a collision currently.
  // if there is, then we need to solve the problem.
  m34 const zu = z - u;
  // there exist the collision.
  // step 1: find the seperating plane.
  math::RealVector3 normal = math::normalized(math::cross(zu.col(2) - zu.col(1), zu.col(3) - zu.col(1)));
  math::RealVector3 center = 0.25 * (zu.col(0) + zu.col(1) + zu.col(2) + zu.col(3));
  real c = math::dot(normal, center);
  real e = math::dot(normal, zu.col(0)) - c;
  if (e < 0) {
    normal = -normal;
    c = -c;
    e = -e;
  }

  real x0c = math::dot(normal, o.col(0)) - c;
  // project to the plane.
  auto proj = [&](math::RealVector3 const& p) -> math::RealVector3 {
    return p - math::dot(normal, p - center) * normal;
  };
  for (Index i = 0; i < 4; ++i) {
    x.col(i) = proj(zu.col(i));
  }

  if (x0c > 0) {
    // step 2: solve edge.
    if (e >= tol - math::epsilon<>) {
      x = zu;
      return false;
    }
    real l = (k * tol + rho * e) / (k + rho);
    if (l <= 0.5 * tol) {
      // We need to enforce the constraint strictly, l >= 0.5 * tol.
      // (k * t + r e) > 0.5 t (k + rho) => 0.5 k t < r e - 0.5 t rho
      k = 2 * (rho * e - 0.5 * tol * rho) / (0.5 * tol);
      l = (k * tol + rho * e) / (k + rho);
    }
    x.col(0) += l * normal;
    x.col(1) -= l * normal / 3;
    x.col(2) -= l * normal / 3;
    x.col(3) -= l * normal / 3;
  } else {
    // step 2: solve vertex.
    real l = (k * -tol + rho * e) / (k + rho);
    if (l >= -0.5 * tol) {
      // We need to enforce the constraint strictly, l <= -0.5 * tol.
      // (k * (-t) + r e) < -0.5 t (k + rho) => 0.5 k t > r e + 0.5 t rho
      k = 2 * (rho * e + 0.5 * tol * rho) / (0.5 * tol);
      l = (k * -tol + rho * e) / (k + rho);
    }

    x.col(0) += l * normal;
    x.col(1) -= l * normal / 3;
    x.col(2) -= l * normal / 3;
    x.col(3) -= l * normal / 3;
  }
  return true;
}

inline bool relax_edge_edge_impl(m34 const& z, m34 const& u, m34 const& o, m34& x, real rho,
                                 real& k, real tol) {
  m34 const zu = z - u;
  math::RealVector3 normal = math::normalized(math::cross(zu.col(1) - zu.col(0), zu.col(3) - zu.col(2)));
  // TODO: If normal is zero.
  math::RealVector3 center = 0.25 * (zu.col(0) + zu.col(1) + zu.col(2) + zu.col(3));
  real c = math::dot(normal, center);
  real e = math::dot(normal, zu.col(3)) - c;
  if (e < 0) {
    normal = -normal;
    c = -c;
    e = -e;
  }
  auto proj = [&](auto const& p) { return p - math::dot(normal, p - center) * normal; };

  for (Index i = 0; i < 4; ++i) {
    x.col(i) = proj(zu.col(i));
  }

  if (math::dot(normal, o.col(3)) - c > 0) {
    if (e >= tol - math::epsilon<>) {
      x = zu;
      return false;
    }

    real l = (k * tol + rho * e) / (k + rho);
    x.col(0) += l * normal;
    x.col(1) += l * normal;
    x.col(2) -= l * normal;
    x.col(3) -= l * normal;
  } else {
    real l = (k * -tol + rho * e) / (k + rho);
    if (l >= -0.5 * tol) {
      k = (rho * e / tol) * 3;
      l = (k * -tol + rho * e) / (k + rho);
    }

    x.col(0) -= l * normal;
    x.col(1) -= l * normal;
    x.col(2) += l * normal;
    x.col(3) += l * normal;
  }

  return true;
}

inline bool relax_vertex_edge_impl(m3 const& z, m3 const& u, m3 const& o, m3& x, real rho, real& k,
                                   real tol) {
  // relax vertex edge.
  // First compute the normal of target positions.
  m3 const zu = z - u;
  math::RealVector3 const e1 = zu.col(1), e2 = zu.col(2), v = zu.col(0);
  math::RealVector3 const n_unnormalized = math::cross(zu.col(1) - zu.col(0), zu.col(2) - zu.col(0));
  math::RealVector3 const o_unnormalized = math::cross(o.col(1) - o.col(0), o.col(2) - o.col(0));
  math::RealVector3 const normal = n_unnormalized.squaredNorm() < 1e-9 ? math::normalized(o_unnormalized)
                                                                 : math::normalized(n_unnormalized);

  std::cout << normal.transpose() << std::endl;
  // Question: How to determine if there is a collision in actual?
  // The collision is determined by the normal of the triangle.
  // if there exist the collision, we need to project to another direction.
  bool const collision = n_unnormalized.dot(o_unnormalized) <= 0;

  // compute the unsigned distance of target position.
  real const area2 = math::norm(math::cross(e1 - v, e2 - v));
  real const distance_v_e = area2 / math::norm(e1 - e2);
  if (distance_v_e > tol && !collision) {
    // ok, you can step to zu directly.
    x = zu;
    return false;
  }

  // the coordinate is y = e2 -> e1, with a normal as z, and the center of mass as origin.
  math::RealVector3 const y_c = math::normalized(e1 - e2);
  math::RealVector3 const& z_c = normal;
  math::RealVector3 const x_c = math::cross(y_c, z_c);
  math::RealVector3 const center_of_mass = (e1 + e2 + v) / 3.0;
  auto to = [&](math::RealVector3 const& p) -> math::RealVector2 {
    return {math::dot(p - center_of_mass, x_c), math::dot(p - center_of_mass, y_c)};
  };

  auto from = [&](math::RealVector2 const& p) -> math::RealVector3 {
    return center_of_mass + p[0] * x_c + p[1] * y_c;
  };

  math::RealVector2 e1_c = to(e1), e2_c = to(e2), v_c = to(v);
  std::cout << "n dot o: " << n_unnormalized.dot(o_unnormalized) << std::endl;
  std::cout << "x: " << x_c.transpose() << std::endl;
  std::cout << "y: " << y_c.transpose() << std::endl;
  std::cout << "n: " << normal.transpose() << std::endl;
  std::cout << "e1c: " << e1_c.transpose() << std::endl;
  std::cout << "e2c: " << e2_c.transpose() << std::endl;
  std::cout << "vc: " << v_c.transpose() << std::endl;
  // e1_c and e2_c should have same x component, and v_c should have different x component.
  if (collision) {
    // output position of e1 and e2 should have different sign compared with e1_c and e2_c.
    real const vcx = v_c.x();
    bool const sgn_vcx = std::signbit(vcx);
    real const t = sgn_vcx ? tol : -tol;
    // k/2(x-t)^2 + rho/2 (x-vcx)^2
    // => optimal point is (k+rho) x = k t + rho vcx
    // => x = (kt + rho vcx) / (k+rho)
    // x represents the optimal x coordinate of vertex in 2D plane.
    real x = (k * t + rho * vcx) / (k + rho);
    // also need to test, if x have same sign with t.
    if (abs(x) < 0.5 * abs(t) || std::signbit(x) == sgn_vcx) {
      // need to enlarge k to satisfy the constraint.
      //   if t < 0 => x < 0.5 t.
      //   if t > 0 => x > 0.5 t.
      // in both case, the threshold is 0.5 t:
      //   (k+rho) 0.5 t = k t + rho vcx.
      // => 0.5 k t = 0.5 rho t - rho vcx.
      // => k = rho - 2 vcx / t.
      // vcx always have different sign with t, then k is always positive.
      k = rho - 2 * vcx / t;
      std::cout << "enlarge k: " << k << std::endl;
      x = (k * t + rho * vcx) / (k + rho);
    }

    // now, we have the optimal x coordinate of vertex, the x coord of edge points are just
    // -0.5 x
    e1_c[0] = -0.5 * x;
    e2_c[0] = -0.5 * x;
    v_c[0] = x;
    std::cout << "collision: " << x << std::endl;
  } else {
    // although there is no collision actually, but the distance is too small.
    // we need to project to a safer range.
    // k/2(x-t)^2 + rho/2 (x-vcx)^2
    real const vcx = v_c.x();
    if (vcx >= tol * 0.9) {
      // no no no, you are ok, allow to set zu.
      x = zu;
      return false;
    }
    bool const sgnb = std::signbit(vcx);
    real const t = sgnb ? -tol : tol;
    real x = (k * t + rho * vcx) / (k + rho);
    // wish |x| > 0.5 |t|
    // => k t + rho vcx = 0.5 t (k + rho)
    // => 0.5 k t = 0.5 rho t - rho vcx
    // => k = rho - 2 vcx / t
    if (abs(x) < 0.5 * abs(t)) {
      k = rho - 2 * vcx / t;
      x = (k * t + rho * vcx) / (k + rho);
    }
    v_c[0] = x;
    e1_c[0] = -0.5 * x;
    e2_c[0] = -0.5 * x;
    std::cout << "no collision: " << x << std::endl;
  }

  // now recover from 2D plane.
  std::cout << "After relax, plane coords:" << std::endl;
  std::cout << "v: " << v_c.transpose() << std::endl;
  std::cout << "e1: " << e1_c.transpose() << std::endl;
  std::cout << "e2: " << e2_c.transpose() << std::endl;
  x.col(0) = from(v_c);
  x.col(1) = from(e1_c);
  x.col(2) = from(e2_c);
  std::cout << x << std::endl;
  std::cout << zu << std::endl;
  // std::cout << o << std::endl;
  return true;
}

inline bool relax_vertex_vertex_impl(m32 const& z, m32 const& u, m32 const& o, m32& dual, real &k, real rho,
                                     real radius, real eps) {
  // Very similar to spring, let L = radius + eps.
  // The spring energy:
  //   f(x) = 1/2 k (|| D x || - (L + eps))^2
  // with Augmented Lagrangian:
  //   f(x) + 1/2 rho || x - z + u ||^2
  // Take derivative:
  //   D.T k (|Dx| - (L + eps)) (Dx)/|Dx| + rho (x - z + u) = 0
  // Apply D again, and take norm.
  //   k(|Dx| - (L + eps)) + rho |Dx| = rho |Dz - Du|
  // => |Dx| = (rho |Dz - Du| + k (L + eps)) / (k + rho)
  // we expect |Dx| >= L.
  //   k (L + eps) + rho |Dz - Du| >= (k + rho) L
  //   k eps >= rho (L - |Dz - Du|)
  // if |Dz - Du| > L, nothing will be relaxed.
  m32 const zu = z - u;  // (z - u)
  v3 const c = 0.5 * (zu.col(0) + zu.col(1));
  v3 const d = zu.col(0) - zu.col(1);

  real const d_norm = d.norm();
  // std::cout << "d_norm: " << d_norm << std::endl;
  if (d_norm >= radius + eps) {
    dual = zu;
    return false;
  }

  real dx_norm = (rho * d_norm + k * (radius + eps)) / (k + rho);
  if (dx_norm < radius) {
    k = 4 * rho * (radius - d_norm) / eps;
    dx_norm = (rho * d_norm + k * (radius + eps)) / (k + rho);
    // std::cout << "relaxing" << k << std::endl;
  }
  // std::cout << "dx_norm: " << dx_norm << std::endl;

  v3 const dn = math::normalized(d);

  if ((zu.col(1) - zu.col(0)).dot(o.col(1) - o.col(0)) < 0) {
    dx_norm = -dx_norm;
  }

  dual.col(0) = c + 0.5 * dx_norm * dn;
  dual.col(1) = c - 0.5 * dx_norm * dn;
  // std::cout << "dual: " << dual << std::endl;
  // std::cout << "z: " << zu << std::endl;
  return true;
}
}  // namespace ax::xpbd
