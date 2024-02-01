#pragma once

#include <cstddef>
#include <cstdint>

namespace ax {

using size_t = std::size_t;  ///< Alias for size type.
using idx = std::ptrdiff_t;  ///< Alias for index type.
using i32 = std::int32_t;    ///< Alias for 32-bit signed integer.
using ui32 = std::uint32_t;  ///< Alias for 32-bit unsigned integer.
using i64 = std::int64_t;    ///< Alias for 64-bit signed integer.
using ui64 = std::uint64_t;  ///< Alias for 64-bit unsigned integer.

using f32 = float;   ///< Alias for single precision floating point number.
using f64 = double;  ///< Alias for double precision floating point number.
using real = f64;    ///< Alias for double precision floating point number.

#ifndef REAL_PRID
#  define REAL_PRID "lf"  ///< The printf format string for Real.
#endif

}  // namespace ax
