#pragma once
#include <fmt/format.h>

#include "common.hpp"
#include "traits.hpp"

template <typename Scalar, int dim> struct fmt::formatter<ax::math::vec<Scalar, dim>> {
  constexpr auto parse(format_parse_context& ctx) /* NOLINT */ { return ctx.begin(); }

  template <typename FormatContext>
  constexpr auto format(const ax::math::vec<Scalar, dim>& v, /* NOLINT */
                        FormatContext& ctx) const {
    fmt::format_to(ctx.out(), "vec<{}, {}>[", Scalar{}, dim);
    for (int i = 0; i < dim; ++i) {
      fmt::format_to(ctx.out(), "{}", v[i]);
      if (i != dim - 1) fmt::format_to(ctx.out(), ", ");
    }
    return fmt::format_to(ctx.out(), "]");
  }
};

template <typename Scalar, int rows, int cols>
struct fmt::formatter<ax::math::mat<Scalar, rows, cols>> {
  constexpr auto parse(format_parse_context& ctx) /* NOLINT */ { return ctx.begin(); }

  template <typename FormatContext>
  constexpr auto format(const ax::math::mat<Scalar, rows, cols>& m, /* NOLINT */
                        FormatContext& ctx) const {
    fmt::format_to(ctx.out(), "mat<{}, {}, {}>[", Scalar{}, rows, cols);
    for (int i = 0; i < rows; ++i) {
      fmt::format_to(ctx.out(), "[");
      for (int j = 0; j < cols; ++j) {
        fmt::format_to(ctx.out(), "{}", m(i, j));
        if (j != cols - 1) fmt::format_to(ctx.out(), ", ");
      }
      fmt::format_to(ctx.out(), "]\n");
      if (i != rows - 1) fmt::format_to(ctx.out(), ", ");
    }
    return fmt::format_to(ctx.out(), "]");
  }
};
