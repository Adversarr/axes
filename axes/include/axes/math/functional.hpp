#pragma once

#include "./common.hpp"
#include "axes/math/details/rsqrt_impl.hpp"
#include "axes/math/traits.hpp"
namespace ax::math {

/****************************** Unary op for scalar ******************************/

template <typename Scalar, typename = enable_if_scalar_t<Scalar>>
AX_FORCE_INLINE auto sqrt(Scalar x) {
  return details::sqrt_impl(x);
}
template <typename Scalar, typename = enable_if_scalar_t<Scalar>>
AX_FORCE_INLINE auto rsqrt(Scalar x) {
  return details::rsqrt_impl(x);
}

#define IMPLEMENT_UNARY_STL(op)                                     \
  template <typename Scalar, typename = enable_if_scalar_t<Scalar>> \
  AX_FORCE_INLINE auto op(Scalar x) {                             \
    return std::op(x);                                              \
  }

IMPLEMENT_UNARY_STL(abs)
IMPLEMENT_UNARY_STL(sin)
IMPLEMENT_UNARY_STL(cos)
IMPLEMENT_UNARY_STL(tan)
IMPLEMENT_UNARY_STL(asin)
IMPLEMENT_UNARY_STL(acos)
IMPLEMENT_UNARY_STL(atan)
IMPLEMENT_UNARY_STL(sinh)
IMPLEMENT_UNARY_STL(cosh)
IMPLEMENT_UNARY_STL(tanh)
IMPLEMENT_UNARY_STL(asinh)
IMPLEMENT_UNARY_STL(acosh)
IMPLEMENT_UNARY_STL(atanh)
IMPLEMENT_UNARY_STL(exp)
IMPLEMENT_UNARY_STL(log)
IMPLEMENT_UNARY_STL(log10)
IMPLEMENT_UNARY_STL(log1p)
IMPLEMENT_UNARY_STL(rint)
IMPLEMENT_UNARY_STL(round)
IMPLEMENT_UNARY_STL(floor)
IMPLEMENT_UNARY_STL(ceil)
IMPLEMENT_UNARY_STL(conj)
IMPLEMENT_UNARY_STL(real)
IMPLEMENT_UNARY_STL(imag)
IMPLEMENT_UNARY_STL(arg)

#undef IMPLEMENT_UNARY_STL

template <typename Scalar, typename = enable_if_scalar_t<Scalar>>
AX_FORCE_INLINE auto cube(Scalar x) {
  return x * x * x;
}

template <typename Scalar, typename = enable_if_scalar_t<Scalar>>
AX_FORCE_INLINE auto abs2(Scalar x) {
  return x * x;
}

template <typename Scalar, typename = enable_if_scalar_t<Scalar>>
AX_FORCE_INLINE auto square(Scalar x) {
  return x * x;
}

template <typename Scalar, typename = enable_if_scalar_t<Scalar>>
AX_FORCE_INLINE auto inverse(Scalar x) {
  return Scalar(1) / x;
}

/****************************** Unary op available for matrices ******************************/
#define IMPLEMENT_UNARY(FUNC, OP)                                                               \
  template <typename Derived> AX_FORCE_INLINE auto FUNC(MBcr<Derived> mv) { return mv.OP(); } \
  template <typename Derived> AX_FORCE_INLINE auto FUNC(ABcr<Derived> mv) { return mv.FUNC(); }
#define A_OP_M(OP) array().OP().matrix()
#define IMPLEMENT_AOPM_UNARY(FUNC) IMPLEMENT_UNARY(FUNC, A_OP_M(FUNC))

IMPLEMENT_UNARY(abs, cwiseAbs)
IMPLEMENT_UNARY(inverse, cwiseInverse)
IMPLEMENT_UNARY(conjugate, conjugate)
IMPLEMENT_UNARY(arg, cwiseArg)

/****************************** Exponential functions ******************************/
IMPLEMENT_AOPM_UNARY(exp)
IMPLEMENT_AOPM_UNARY(log)
IMPLEMENT_AOPM_UNARY(log1p)
IMPLEMENT_AOPM_UNARY(log10)

/****************************** Power functions ******************************/
IMPLEMENT_UNARY(sqrt, cwiseSqrt)
IMPLEMENT_AOPM_UNARY(rsqrt)
IMPLEMENT_AOPM_UNARY(square)
IMPLEMENT_AOPM_UNARY(cube)
IMPLEMENT_AOPM_UNARY(abs2)

/****************************** Trigonometric functions ******************************/
IMPLEMENT_AOPM_UNARY(sin)
IMPLEMENT_AOPM_UNARY(cos)
IMPLEMENT_AOPM_UNARY(tan)
IMPLEMENT_AOPM_UNARY(asin)
IMPLEMENT_AOPM_UNARY(acos)
IMPLEMENT_AOPM_UNARY(atan)

/****************************** Hyperbolic functions ******************************/
IMPLEMENT_AOPM_UNARY(sinh)
IMPLEMENT_AOPM_UNARY(cosh)
IMPLEMENT_AOPM_UNARY(tanh)
IMPLEMENT_AOPM_UNARY(asinh)
IMPLEMENT_AOPM_UNARY(acosh)
IMPLEMENT_AOPM_UNARY(atanh)

/****************************** Nearest integer ******************************/
IMPLEMENT_AOPM_UNARY(round)
IMPLEMENT_AOPM_UNARY(floor)
IMPLEMENT_AOPM_UNARY(ceil)
IMPLEMENT_AOPM_UNARY(rint)

/****************************** Classification and comparison ******************************/
IMPLEMENT_AOPM_UNARY(isFinite)
IMPLEMENT_AOPM_UNARY(isInf)
IMPLEMENT_AOPM_UNARY(isNan)  // TODO: Naming is bad.

#undef IMPLEMENT_AOPM_UNARY
#undef IMPLEMENT_UNARY

template <typename Derived> AX_FORCE_INLINE Derived::Scalar sum(DBcr<Derived> a) {
  return a.sum();
}

template <typename Derived> AX_FORCE_INLINE Derived::Scalar prod(DBcr<Derived> a) {
  return a.prod();
}

template <typename Derived> AX_FORCE_INLINE Derived::Scalar mean(DBcr<Derived> a) {
  return a.mean();
}

template <typename A> AX_FORCE_INLINE A::ScalarType max(DBcr<A> mv) { return mv.maxCoeff(); }

template <typename A> AX_FORCE_INLINE A::ScalarType min(DBcr<A> mv) { return mv.minCoeff(); }

template <typename A> AX_FORCE_INLINE A::ScalarType trace(DBcr<A> mv) { return mv.trace(); }

template <typename A> AX_FORCE_INLINE bool all(DBcr<A> mv) {
  static_assert(std::is_same_v<typename A::ScalarType, bool>,
                "all() is only available for bool vectors");
  return mv.all();
}

template <typename A> AX_FORCE_INLINE bool any(DBcr<A> mv) {
  static_assert(std::is_same_v<typename A::ScalarType, bool>,
                "all() is only available for bool vectors");
  return mv.any();
}

template <typename A> AX_FORCE_INLINE idx count(DBcr<A> mv) {
  static_assert(std::is_same_v<typename A::ScalarType, bool>,
                "all() is only available for bool vectors");
  return mv.count();
}

/****************************** argxxx ******************************/

template <typename Derived, typename = std::enable_if_t<cols_v<Derived> == 1>>
AX_FORCE_INLINE idx argmax(DBcr<Derived> mv) {
  idx coef = -1;
  mv.maxCoeff(coef);
  return coef;
}

template <typename Derived, typename = std::enable_if_t<cols_v<Derived> == 1>>
AX_FORCE_INLINE idx argmin(DBcr<Derived> mv) {
  idx coef = -1;
  mv.minCoeff(coef);
  return coef;
}

}  // namespace ax::math
