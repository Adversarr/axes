#pragma once

#include <random>

#include "./common.hpp"

#ifndef __CUDACC__
#  include "ax/math/details/rsqrt_impl.hpp"
#endif

#include "ax/math/traits.hpp"
namespace ax::math {

/****************************** Unary op for scalar ******************************/
#ifdef __CUDACC__

template <typename Scalar, typename = enable_if_scalar_t<Scalar>>
AX_HOST_DEVICE AX_FORCE_INLINE auto sqrt(Scalar x) {
  return ::sqrt(x);
}
template <typename Scalar, typename = enable_if_scalar_t<Scalar>>
AX_HOST_DEVICE AX_FORCE_INLINE auto rsqrt(Scalar x) {
  return ((Scalar)1.0) / ::sqrt(x);
}

#else

template <typename Scalar, typename = enable_if_scalar_t<Scalar>>
AX_FORCE_INLINE auto sqrt(Scalar x) {
  return details::sqrt_impl(x);
}
template <typename Scalar, typename = enable_if_scalar_t<Scalar>>
AX_FORCE_INLINE auto rsqrt(Scalar x) {
  return details::rsqrt_impl(x);
}

#endif

template <typename Scalar, typename = enable_if_scalar_t<Scalar>>
AX_HOST_DEVICE AX_FORCE_INLINE Scalar radians(Scalar degree) {
  return degree * pi_radian<Scalar>;
}

#define IMPLEMENT_UNARY_STL(op)                                     \
  template <typename Scalar, typename = enable_if_scalar_t<Scalar>> \
  AX_HOST_DEVICE AX_FORCE_INLINE auto op(Scalar x) {                \
    return std::op(x);                                              \
  }

template <typename Scalar, typename = enable_if_scalar_t<Scalar>>
AX_HOST_DEVICE AX_FORCE_INLINE Scalar abs(Scalar x) {
  return std::fabs(x);
}

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
#undef IMPLEMENT_UNARY_STL

template <typename Scalar, typename = enable_if_scalar_t<Scalar>>
AX_HOST_DEVICE AX_FORCE_INLINE auto pow(Scalar x, Scalar y) {
  return std::pow(x, y);
}

template <typename Scalar, typename = enable_if_scalar_t<Scalar>>
AX_HOST_DEVICE AX_FORCE_INLINE auto cube(Scalar x) {
  return x * x * x;
}

template <typename Scalar, typename = enable_if_scalar_t<Scalar>>
AX_HOST_DEVICE AX_FORCE_INLINE auto abs2(Scalar x) {
  return x * x;
}

template <typename Scalar, typename = enable_if_scalar_t<Scalar>>
AX_HOST_DEVICE AX_FORCE_INLINE auto square(Scalar x) {
  return x * x;
}

template <typename Scalar, typename = enable_if_scalar_t<Scalar>>
AX_HOST_DEVICE AX_FORCE_INLINE auto inverse(Scalar x) {
  return Scalar(1) / x;
}

/****************************** Unary op available for matrices ******************************/
#define IMPLEMENT_UNARY(FUNC, OP)                                                          \
  template <typename Derived, typename = std::enable_if_t<!is_scalar_v<Derived>, Derived>> \
  AX_HOST_DEVICE AX_FORCE_INLINE auto FUNC(MBcr<Derived> mv) {                             \
    return mv.OP();                                                                        \
  }                                                                                        \
  template <typename Derived, typename = std::enable_if_t<!is_scalar_v<Derived>, Derived>> \
  AX_HOST_DEVICE AX_FORCE_INLINE auto FUNC(ABcr<Derived> mv) {                             \
    return mv.FUNC();                                                                      \
  }
#define A_OP_M(OP) array().OP().matrix
#define IMPLEMENT_AOPM_UNARY(FUNC) IMPLEMENT_UNARY(FUNC, A_OP_M(FUNC))

IMPLEMENT_UNARY(abs, cwiseAbs)
IMPLEMENT_UNARY(inverse, cwiseInverse)

// NOTE: MSVC+NVCC does not support arg
// IMPLEMENT_UNARY(conjugate, conjugate)
// IMPLEMENT_UNARY(arg, cwiseArg)

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

template <typename Derived>
AX_HOST_DEVICE AX_FORCE_INLINE typename Derived::Scalar sum(DBcr<Derived> a) {
  return a.sum();
}

template <typename Derived>
AX_HOST_DEVICE AX_FORCE_INLINE typename Derived::Scalar prod(DBcr<Derived> a) {
  return a.prod();
}

template <typename Derived>
AX_HOST_DEVICE AX_FORCE_INLINE typename Derived::Scalar mean(DBcr<Derived> a) {
  return a.mean();
}

template <typename A> AX_HOST_DEVICE AX_FORCE_INLINE typename A::Scalar max(DBcr<A> mv) {
  return mv.maxCoeff();
}

template <typename A> AX_HOST_DEVICE AX_FORCE_INLINE typename A::Scalar min(DBcr<A> mv) {
  return mv.minCoeff();
}

template <typename A> AX_HOST_DEVICE AX_FORCE_INLINE typename A::Scalar trace(MBcr<A> mv) {
  return mv.trace();
}

template <typename A> AX_HOST_DEVICE AX_FORCE_INLINE bool all(DBcr<A> mv) { return mv.all(); }

template <typename A> AX_HOST_DEVICE AX_FORCE_INLINE bool any(DBcr<A> mv) { return mv.any(); }

template <typename A> AX_HOST_DEVICE AX_FORCE_INLINE Index count(DBcr<A> mv) { return mv.count(); }

/****************************** argxxx ******************************/

template <typename Derived, typename = std::enable_if_t<cols_v<Derived> == 1>>
AX_HOST_DEVICE AX_FORCE_INLINE Index argmax(DBcr<Derived> mv) {
  Index coef = -1;
  mv.maxCoeff(coef);
  return coef;
}

template <typename Derived, typename = std::enable_if_t<cols_v<Derived> == 1>>
AX_HOST_DEVICE AX_FORCE_INLINE Index argmin(DBcr<Derived> mv) {
  Index coef = -1;
  mv.minCoeff(coef);
  return coef;
}

/****************************** Specials ******************************/
using std::clamp;
using std::fmod;
using std::isfinite;
using std::isinf;
using std::isnan;
using std::lgamma;
using std::tgamma;

template <int dim> AX_HOST_DEVICE AX_FORCE_INLINE math::IndexVector<dim> imod(const math::IndexVector<dim>& a,
                                                                       const math::IndexVector<dim>& b) {
  math::IndexVector<dim> output;
#ifdef __clang__
#  pragma unroll
#endif
  for (Index d = 0; d < dim; ++d) {
    output[d] = a[d] % b[d];
  }
  return output;
}

struct subscript_t {};
struct stride_t {};
constexpr subscript_t subscript;
constexpr stride_t stride;

template <int dim> AX_HOST_DEVICE AX_FORCE_INLINE Index sub2ind(math::IndexVector<dim> const& sub,
                                                              math::IndexVector<dim> const& stride,
                                                              stride_t) {
  return dot(sub, stride);
}

template <int dim>
AX_HOST_DEVICE AX_FORCE_INLINE math::IndexVector<dim> to_stride(math::IndexVector<dim> const& shape) {
  math::IndexVector<dim> stride;
  stride[dim - 1] = 1;
  for (Index d = dim - 2; d >= 0; --d) {
    stride[d] = stride[d + 1] * shape[d + 1];
  }
  return stride;
}

template <int dim> AX_HOST_DEVICE AX_FORCE_INLINE Index sub2ind(math::IndexVector<dim> const& sub,
                                                              math::IndexVector<dim> const& shape,
                                                              subscript_t = subscript) {
  return sub2ind(sub, math::to_stride<dim>(shape), stride);
}

template <int dim>
AX_HOST_DEVICE AX_FORCE_INLINE math::IndexVector<dim> ind2sub(Index ind, math::IndexVector<dim> const& stride,
                                                       subscript_t) {
  math::IndexVector<dim> sub;
  for (Index d = dim - 1; d >= 0; --d) {
    sub[d] = ind / stride[d];
    ind -= sub[d] * stride[d];
  }
  return sub;
}

template <typename T, typename A, typename B>
AX_HOST_DEVICE AX_FORCE_INLINE auto lerp(A const& a, B const& b, T const& t) {
  static_assert(is_scalar_v<T>, "T must be scalar.");
  return (static_cast<T>(1) - t) * a + t * b;
}

template <typename Scalar = Real, typename = enable_if_scalar_t<Scalar>>
AX_HOST AX_FORCE_INLINE auto random(Scalar low = 0, Scalar high = 1) {
  std::random_device rand_dev;
  std::default_random_engine rand_gen(rand_dev());
  auto const rand_int = rand_gen();
  auto const rand_range = rand_gen.max() - rand_gen.min();
  auto const rand_scalar = static_cast<Scalar>(rand_int) / static_cast<Scalar>(rand_range);
  return low + (high - low) * rand_scalar;
}

}  // namespace ax::math
