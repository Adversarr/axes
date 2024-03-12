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
AX_FORCE_INLINE auto dot(MBcr<DerivedA> a, MBcr<DerivedB> b) {
  static_assert(DerivedA::RowsAtCompileTime == DerivedB::RowsAtCompileTime,
                "dot product requires vectors of the same size");

  static_assert(DerivedA::ColsAtCompileTime == DerivedB::ColsAtCompileTime,
                "dot product requires vectors of the same size");

  return sum(as_array(a) * as_array(b));
}

template <typename A, typename B>
AX_FORCE_INLINE auto cross(MBcr<A> a, MBcr<B> b) {
  static_assert(A::RowsAtCompileTime == 3, "cross product requires 3D vectors");
  static_assert(B::RowsAtCompileTime == 3, "cross product requires 3D vectors");

  static_assert(A::ColsAtCompileTime == 1, "cross product requires 3D vectors");
  static_assert(B::ColsAtCompileTime == 1, "cross product requires 3D vectors");

  return a.cross(b);
}

/****************************** norms ******************************/

template <typename A> AX_FORCE_INLINE auto norm(MBcr<A> mv, l2_t = {}) {
  return mv.norm();
}

template <typename A> AX_FORCE_INLINE auto norm2(MBcr<A> mv) {
  return mv.squaredNorm();
}

template <typename A> AX_FORCE_INLINE auto norm(MBcr<A> mv, linf_t) {
  return mv.template lpNorm<Eigen::Infinity>();
}

template <typename A> AX_FORCE_INLINE auto norm(MBcr<A> mv, l1_t) {
  return mv.template lpNorm<1>();
}

template <typename A> AX_FORCE_INLINE auto normalized(MBcr<A> mv) {
  return mv.normalized();
}

template <typename A> AX_FORCE_INLINE decltype(auto) normalize_(MBr<A> mv) {
  mv.normalize();
  return mv;
}

/****************************** outer ******************************/
template <typename A, typename B>
AX_FORCE_INLINE auto outer(MBcr<A> a, MBcr<B> b) {
  return a * b.transpose();
}

template <typename A> AX_FORCE_INLINE auto outer(MBcr<A> a) {
  return outer(a, a);
}

/****************************** determinant ******************************/
template <typename A> AX_FORCE_INLINE auto det(MBcr<A> a) {
  return a.determinant();
}

/****************************** inverse and solve
 * ******************************/
// TODO: Add invertible test function.

// WARN: This function may fail. Prefer to use decompositions.
template <typename A> AX_FORCE_INLINE auto inv(MBcr<A> a) {
  return a.inverse();
}

// WARN: This function may fail. Prefer to use decompositions.
template <typename A, typename B>
AX_FORCE_INLINE auto solve(MBcr<A> a, MBcr<B> b) {
  return a.solve(b);
}

/****************************** Pinv and Psolve ******************************/

template <typename A> AX_FORCE_INLINE auto pinv(MBcr<A> a) {
  return a.completeOrthogonalDecomposition().pseudoInverse();
}

template <typename A, typename B>
AX_FORCE_INLINE auto psolve(MBcr<A> a, MBcr<B> b) {
  return a.completeOrthogonalDecomposition().solve(b);
}

/****************************** angle ******************************/
template <typename A, typename B>
AX_FORCE_INLINE auto angle(MBcr<A> a, MBcr<B> b) {
  auto norm_a = norm(a);
  auto norm_b = norm(b);
  auto cos_theta = dot(a, b) / (norm_a * norm_b);
  return math::acos(math::clamp(cos_theta, -1.0, 1.0));
}


}  // namespace ax::math
