#pragma once
#include "ax/core/dim.hpp"
#include "ax/utils/enum_refl.hpp"

namespace ax {

AX_DEFINE_ENUM_CLASS(BufferDevice, Host, Device);

template <typename T>
class Buffer;

// Holds a view over a buffer, does not own the memory.
template <typename T>
class BufferView;

// Buffer dimension type, use dim3 instead of DimVariant for better performance
using BufferDim = Dim3;

}  // namespace ax

namespace fmt {

template <>
struct formatter<ax::BufferDevice> : formatter<std::string_view> {
  template <typename FormatContext>
  auto format(ax::BufferDevice c, FormatContext& ctx) const {
    return formatter<std::string_view>::format(ax::utils::reflect_name(c).value_or("???"), ctx);
  }
};

template <size_t N>
struct formatter<ax::Dim<N>> : formatter<std::string_view> {
  template <typename FormatContext>
  auto format(ax::Dim<N> c, FormatContext& ctx) const {
    fmt::format_to(ctx.out(), "(");
    for (size_t i = 0; i < N - 1; ++i) {
      fmt::format_to(ctx.out(), "{}, ", c.sizes_[i]);
    }
    fmt::format_to(ctx.out(), "{})", c.sizes_[N - 1]);
    return ctx.out();
  }
};

}  // namespace fmt
