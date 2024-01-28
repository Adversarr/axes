#pragma once

#include "./common.hpp"
#include "axes/math/details/rsqrt_impl.hpp"
namespace ax::math {

// Unary Ops: sin, cos, tan, cot, ...

template <typename Scalar> AXES_FORCE_INLINE auto sqrt(Scalar x) { return details::sqrt_impl(x); }
template <typename Scalar> AXES_FORCE_INLINE auto rsqrt(Scalar x) { return details::rsqrt_impl(x); }

}  // namespace ax::math
