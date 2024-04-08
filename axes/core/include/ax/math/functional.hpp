#pragma once

#include "./common.hpp"
#include "ax/math/details/rsqrt_impl.hpp"
#include "ax/math/traits.hpp"
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

template <typename Scalar, typename = enable_if_scalar_t<Scalar>>
AX_FORCE_INLINE Scalar radians(Scalar degree) {
  return degree * pi_radian<Scalar>;
}

#define IMPLEMENT_UNARY_STL(op)                                     \
  template <typename Scalar, typename = enable_if_scalar_t<Scalar>> \
  AX_FORCE_INLINE auto op(Scalar x) {                               \
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
#undef IMPLEMENT_UNARY_STL

template <typename Scalar, typename = enable_if_scalar_t<Scalar>>
AX_FORCE_INLINE auto pow(Scalar x, Scalar y) {
  return std::pow(x, y);
}

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
#define IMPLEMENT_UNARY(FUNC, OP)                                                             \
  template <typename Derived, typename = std::enable_if_t<!is_scalar_v<Derived>, Derived>> \
  AX_FORCE_INLINE auto FUNC(MBcr<Derived> mv) { return mv.OP(); } \
  template <typename Derived, typename = std::enable_if_t<!is_scalar_v<Derived>, Derived>> \
  AX_FORCE_INLINE auto FUNC(ABcr<Derived> mv) { return mv.FUNC(); }
#define A_OP_M(OP) array().OP().matrix
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

template <typename Derived> AX_FORCE_INLINE typename Derived::Scalar sum(DBcr<Derived> a) {
  return a.sum();
}

template <typename Derived> AX_FORCE_INLINE typename Derived::Scalar prod(DBcr<Derived> a) {
  return a.prod();
}

template <typename Derived> AX_FORCE_INLINE typename Derived::Scalar mean(DBcr<Derived> a) {
  return a.mean();
}

template <typename A> AX_FORCE_INLINE typename A::Scalar max(DBcr<A> mv) {
  return mv.maxCoeff();
}

template <typename A> AX_FORCE_INLINE typename A::Scalar min(DBcr<A> mv) {
  return mv.minCoeff();
}

template <typename A> AX_FORCE_INLINE typename A::Scalar trace(MBcr<A> mv) {
  return mv.trace();
}

template <typename A> AX_FORCE_INLINE bool all(DBcr<A> mv) { return mv.all(); }

template <typename A> AX_FORCE_INLINE bool any(DBcr<A> mv) { return mv.any(); }

template <typename A> AX_FORCE_INLINE idx count(DBcr<A> mv) { return mv.count(); }

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

/****************************** Specials ******************************/
using std::clamp;
using std::fmod;
using std::lgamma;
using std::tgamma;
using std::isnan;
using std::isinf;
using std::isfinite;

template <idx dim> math::veci<dim> imod(const math::veci<dim>& a, const math::veci<dim>& b) {
  math::veci<dim> output;
#ifdef __clang__
#pragma unroll
#endif
  for (idx d = 0; d < dim; ++d) {
    output[d] = a[d] % b[d];
  }
  return output;
}

struct subscript_t {};
struct stride_t {};
constexpr subscript_t subscript;
constexpr stride_t stride;

template <idx dim> idx sub2ind(math::veci<dim> const& sub, math::veci<dim> const& stride, stride_t) {
  return dot(sub, stride);
}

template<idx dim> math::veci<dim> to_stride(math::veci<dim> const& shape) {
  math::veci<dim> stride;
  stride[dim - 1] = 1;
  for (idx d = dim - 2; d >= 0; --d) {
    stride[d] = stride[d + 1] * shape[d + 1];
  }
  return stride;
}

template <idx dim> idx sub2ind(math::veci<dim> const& sub, math::veci<dim> const& shape, subscript_t = subscript) {
  return sub2ind(sub, math::to_stride<dim>(shape), stride);
}

template <idx dim> math::veci<dim> ind2sub(idx ind, math::veci<dim> const& stride, subscript_t) {
  math::veci<dim> sub;
  for (idx d = dim - 1; d >= 0; --d) {
    sub[d] = ind / stride[d];
    ind -= sub[d] * stride[d];
  }
  return sub;
}


namespace details {
constexpr real factorials[32] = {1.0,
                                 1.0,
                                 2.0,
                                 6.0,
                                 24.0,
                                 120.0,
                                 720.0,
                                 5040.0,
                                 40320.0,
                                 362880.0,
                                 3628800.0,
                                 39916800.0,
                                 479001600.0,
                                 6227020800.0,
                                 87178291200.0,
                                 1307674368000.0,
                                 20922789888000.0,
                                 355687428096000.0,
                                 6402373705728000.0,
                                 121645100408832000.0,
                                 2432902008176640000.0,
                                 51090942171709440000.0,
                                 1124000727777607680000.0,
                                 25852016738884976640000.0,
                                 620448401733239439360000.0,
                                 15511210043330985984000000.0,
                                 403291461126605635584000000.0,
                                 10888869450418352160768000000.0,
                                 304888344611713860501504000000.0,
                                 8841761993739701954543616000000.0,
                                 265252859812191058636308480000000.0,
                                 8222838654177922817725562880000000.0};
}

}  // namespace ax::math
