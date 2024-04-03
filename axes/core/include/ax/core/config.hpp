#pragma once

#include <cstddef>
#include <cstdint>

namespace ax {

using size_t = std::size_t;  ///< Alias for size type.
using idx = std::ptrdiff_t;  ///< Alias for index type.
using i8 = std::int8_t;      ///< Alias for 8-bit signed integer.
using i16 = std::int16_t;    ///< Alias for 16-bit signed integer.
using i32 = std::int32_t;    ///< Alias for 32-bit signed integer.
using i64 = std::int64_t;    ///< Alias for 64-bit signed integer.
using ui8 = std::uint8_t;    ///< Alias for 8-bit unsigned integer.
using ui16 = std::uint16_t;  ///< Alias for 16-bit unsigned integer.
using ui32 = std::uint32_t;  ///< Alias for 32-bit unsigned integer.
using ui64 = std::uint64_t;  ///< Alias for 64-bit unsigned integer.

using f32 = float;   ///< Alias for single precision floating point number.
using f64 = double;  ///< Alias for double precision floating point number.
using real = f64;    ///< Alias for double precision floating point number.

using EnumUnderlyingType = std::int32_t;  ///< Alias for enum underlying type.

#ifndef REAL_PRID
#  define REAL_PRID "d"  ///< The printf format string for Real.
#endif

}  // namespace ax

#define AX_DECLARE_ENUM(EnumName) enum class EnumName : ax::EnumUnderlyingType
