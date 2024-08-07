#pragma once

#include <Eigen/Geometry>

#include "functional.hpp"

namespace ax::math {

struct l2_t {};
struct linf_t {};
struct l1_t {};
constexpr l2_t l2{};
constexpr linf_t linf{};
constexpr l1_t l1{};

template <typename DerivedA, typename DerivedB>
AX_HOST_DEVICE AX_FORCE_INLINE auto dot(MBcr<DerivedA> a, MBcr<DerivedB> b) {
  static_assert(DerivedA::RowsAtCompileTime == DerivedB::RowsAtCompileTime,
                "dot product requires vectors of the same size");

  static_assert(DerivedA::ColsAtCompileTime == DerivedB::ColsAtCompileTime,
                "dot product requires vectors of the same size");

  return sum(as_array(a) * as_array(b));
}

template <typename A, typename B>
AX_HOST_DEVICE AX_FORCE_INLINE auto cross(MBcr<A> a, MBcr<B> b) {
  static_assert(A::RowsAtCompileTime == 3, "cross product requires 3D vectors");
  static_assert(B::RowsAtCompileTime == 3, "cross product requires 3D vectors");

  static_assert(A::ColsAtCompileTime == 1, "cross product requires 3D vectors");
  static_assert(B::ColsAtCompileTime == 1, "cross product requires 3D vectors");

  return a.cross(b);
}

/****************************** norms ******************************/

template <typename A>
AX_HOST_DEVICE AX_FORCE_INLINE typename A::Scalar norm(MBcr<A> mv, l2_t = {}) {
  return mv.norm();
}

template <typename A> AX_HOST_DEVICE AX_FORCE_INLINE typename A::Scalar norm2(MBcr<A> mv) {
  return mv.squaredNorm();
}

template <typename A> AX_HOST_DEVICE AX_FORCE_INLINE typename A::Scalar norm(MBcr<A> mv, linf_t) {
  return mv.cwiseAbs().maxCoeff();
}

template <typename A> AX_HOST_DEVICE AX_FORCE_INLINE typename A::Scalar norm(MBcr<A> mv, l1_t) {
  return mv.cwiseAbs().sum();
}

template <typename A> AX_HOST_DEVICE AX_FORCE_INLINE auto normalized(MBcr<A> mv) {
  return mv.normalized();
}

template <typename A> AX_HOST_DEVICE AX_FORCE_INLINE void normalize_(MBr<A> mv) { mv.normalize(); }

/****************************** outer ******************************/
template <typename A, typename B> AX_HOST_DEVICE AX_FORCE_INLINE auto outer(MBcr<A> a, MBcr<B> b) {
  return a * b.transpose();
}

template <typename A> AX_HOST_DEVICE AX_FORCE_INLINE auto outer(MBcr<A> a) { return outer(a, a); }

/****************************** determinant ******************************/
template <typename A> AX_HOST_DEVICE AX_FORCE_INLINE typename A::Scalar det(MBcr<A> a) {
  return a.determinant();
}

/****************************** inverse and solve ******************************/
// TODO: Add invertible test function.

// WARN: This function may fail. Prefer to use decompositions.
template <typename A> AX_FORCE_INLINE auto inv(MBcr<A> a) { return a.inverse(); }

// WARN: This function may fail. Prefer to use decompositions.
template <typename A, typename B> AX_FORCE_INLINE auto solve(MBcr<A> a, MBcr<B> b) {
  return a.solve(b);
}

/****************************** Pinv and Psolve ******************************/

template <typename A> AX_FORCE_INLINE auto pinv(MBcr<A> a) {
  return a.completeOrthogonalDecomposition().pseudoInverse();
}

template <typename A, typename B> AX_FORCE_INLINE auto psolve(MBcr<A> a, MBcr<B> b) {
  return a.completeOrthogonalDecomposition().solve(b);
}

/****************************** angle ******************************/
template <typename A, typename B>
AX_HOST_DEVICE AX_FORCE_INLINE typename A::Scalar angle(MBcr<A> a, MBcr<B> b) {
  static_assert(A::ColsAtCompileTime == 1, "angle requires vectors(A)");
  static_assert(B::ColsAtCompileTime == 1, "angle requires vectors(B)");

  auto norm_a = norm(a);
  auto norm_b = norm(b);
  auto cos_theta = dot(a, b) / (norm_a * norm_b);
  return acos(clamp(cos_theta, -1.0, 1.0));
}

/****************************** eig ******************************/
template <typename A> AX_HOST_DEVICE AX_FORCE_INLINE
    std::pair<math::mat<typename A::Scalar, A::RowsAtCompileTime, A::RowsAtCompileTime>,
              math::vec<typename A::Scalar, A::RowsAtCompileTime>>
    eig(MBcr<A> a) {
  static_assert(A::RowsAtCompileTime == A::ColsAtCompileTime, "eig requires square matrix");
  Eigen::SelfAdjointEigenSolver<A> es(a);
  return std::make_pair(es.eigenvectors(), es.eigenvalues());
}

template <typename A, typename EVec, typename EVal>
AX_HOST_DEVICE AX_FORCE_INLINE void eig(MBcr<A> a, MBr<EVec> e_vector, MBr<EVal> e_value) {
  static_assert(A::RowsAtCompileTime == A::ColsAtCompileTime, "eig requires square matrix");
  static_assert(EVec::ColsAtCompileTime == EVec::ColsAtCompileTime,
                "Eigen vector matrix row != col");
  Eigen::SelfAdjointEigenSolver<A> es(a);
  e_vector = es.eigenvectors();
  e_value = es.eigenvalues();
}

/****************************** barycentric ******************************/
AX_HOST_DEVICE AX_FORCE_INLINE math::vec2r barycentric(math::vec2r const& p, math::vec2r const& a,
                                                       math::vec2r const& b) {
  math::vec2r v0 = b - a, v1 = p - a;
  real d00 = math::dot(v0, v0);
  real d01 = math::dot(v0, v1);
  real denom = d00;
  real v = d01 / denom;
  real u = 1.0 - v;
  return math::vec2r(u, v);
}

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

/****************************** lerp ******************************/
template <typename A, typename B, typename Scalar>
AX_HOST_DEVICE AX_FORCE_INLINE auto lerp(MBcr<A> a, MBcr<B> b, Scalar t) {
  return a + t * (b - a);
}

}  // namespace ax::math
