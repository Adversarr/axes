/**
 * @file
 * @brief AXES Core module:
 * 1. Program sequence control: computational graph sub-module;
 * 2. Data storaging: ECS sub-module;
 * 3. Math library: Eigen3 based math sub-module;
 */
#pragma once

#include <cstddef>
#include <cstdint>

namespace axes {

using size_t = std::size_t;    ///< Alias for size type.
using Index = std::ptrdiff_t;  ///< Alias for index type.
using Int32 = std::int32_t;    ///< Alias for 32-bit signed integer.
using UInt32 = std::uint32_t;  ///< Alias for 32-bit unsigned integer.
using Int64 = std::int64_t;    ///< Alias for 64-bit signed integer.
using UInt64 = std::uint64_t;  ///< Alias for 64-bit unsigned integer.

using Float = double;          ///< Alias for double precision floating point number.
using Float32 = float;         ///< Alias for single precision floating point number.
using Float64 = double;        ///< Alias for double precision floating point number.
using Float128 = long double;  ///< Alias for extended floating point number.
using Real = double;           ///< Alias for double precision floating point number.
using GpuReal = float;         ///< Alias for single precision floating point number.

#ifndef REAL_PRID
#  define REAL_PRID "lf"  ///< The printf format string for Real.
#endif

}  // namespace axes
