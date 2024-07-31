#pragma once
#include <fmt/format.h>
#include <fmt/ostream.h>

#include "ax/math/common.hpp"  // IWYU pragma: export

// Not column vector
template <typename T> struct fmt::formatter<
    T, std::enable_if_t<std::is_base_of_v<Eigen::DenseBase<T>, T>
                            && !(T::ColsAtCompileTime == 1 || T::RowsAtCompileTime != 1),
                        char>> : ostream_formatter {};
// A column vector
template <typename T> struct fmt::formatter<
    T, std::enable_if_t<std::is_base_of_v<Eigen::DenseBase<T>, T>
                            && (T::ColsAtCompileTime == 1 || T::RowsAtCompileTime != 1),
                        char>> {
  constexpr auto parse(format_parse_context& ctx) /* NOLINT */ { return ctx.begin(); }

  template <typename FormatContext>
  auto format(T const& vec, FormatContext& ctx) /* NOLINT */ const {
    return ostream_formatter{}.format(vec.transpose(), ctx);
  }
};
