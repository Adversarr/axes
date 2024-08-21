#pragma once
#include <fmt/format.h>
#include <fmt/ostream.h>

#include "../common.hpp"
#include "../shape.hpp"
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

template <typename IndexType, int dim> struct fmt::formatter<ax::math::ShapeArray<IndexType, dim>> {
  constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  constexpr auto format(const ax::math::ShapeArray<IndexType, dim>& p, FormatContext& ctx) const {
    format_to(ctx.out(), "(");
    for (int i = 0; i < dim; ++i) {
      format_to(ctx.out(), "{}", p.data_[i]);
      if (i + 1 < dim) {
        format_to(ctx.out(), ", ");
      }
    }
    return format_to(ctx.out(), ")");
  }
};
