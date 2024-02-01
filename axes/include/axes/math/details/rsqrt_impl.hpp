#pragma once
#include <cmath>

#include "axes/core/common.hpp"

#if defined(__i386__) || defined(__x86_64__)
#  include <immintrin.h>
#  define AX_RSQRT32(x) _mm_cvtss_f32(_mm_rsqrt_ss(_mm_set_ss((x))))
#endif

#if defined(__arm64__)
#  include <arm_neon.h>
#  define AX_RSQRT32(x) vrsqrtes_f32((x))
#  define AX_RSQRT64(x) vrsqrted_f64((x))
#  define AX_SQRT64(x) vsqrt_f64((x))
#endif

namespace ax::math::details {

inline f32 rsqrt_impl(f32 input) {
#ifdef AX_RSQRT32
  return AX_RSQRT32(input);
#else
  // Refer to https://en.wikipedia.org/wiki/Fast_inverse_square_root
  long i;
  float x2, y;
  const float threehalfs = 1.5F;
  x2 = input * 0.5F;
  y = input;
  i = *(long *)&y;            // evil floating point bit level hacking
  i = 0x5f3759df - (i >> 1);  // what the fuck?
  y = *(float *)&i;
  y = y * (threehalfs - (x2 * y * y));  // 1st iteration
  return y;
#endif
}

inline f64 rsqrt_impl(f64 input) {
#ifdef AX_RSQRT64
  return AX_RSQRT64(input);
#else
  return 1 / std::sqrt(input);
#endif
}

inline f32 sqrt_impl(f32 x) { return std::sqrt(x); }

inline f64 sqrt_impl(f64 input) {
#ifdef AX_SQRT64
  float64x1_t x;
  x[0] = input;
  float64x1_t r = AX_SQRT64(x);
  return r[0];
#else
  return std::sqrt(input);
#endif
}

}  // namespace ax::math::details
