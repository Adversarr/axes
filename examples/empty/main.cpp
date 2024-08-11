#include <fmt/ranges.h>
#include <spdlog/fmt/std.h>

#include <ax/math/shape.hpp>

#include "ax/core/init.hpp"
#include "ax/core/logging.hpp"
#include "ax/math/accessor.hpp"
#include "ax/math/common.hpp"
#include "ax/math/field.hpp"
#include "ax/math/views.hpp"
#include "ax/utils/time.hpp"

using namespace ax;

static void benchmark_mm(int scale) {
  int rows = 1 << scale;
  math::mat<f32, math::dynamic, math::dynamic> m1(rows, rows);
  math::mat<f32, math::dynamic, math::dynamic> m2(rows, rows);
  m1.setRandom();
  m2.setRandom();

  auto start = utils::now();
  (m1 * m2).eval();
  auto end = utils::now();
  AX_INFO("Matrix multiplication: {} x {}, {}", rows, rows,
          std::chrono::duration_cast<utils::milliseconds>(end - start));
}

static void benchmark_transpose_inplace(int scale) {
  int rows = 1 << scale;
  math::mat<f32, math::dynamic, math::dynamic> m(rows, rows);
  m.setRandom();

  auto start = utils::now();
  m.transposeInPlace();
  auto end = utils::now();
  AX_INFO("Transpose inplace: {} x {}, {}", rows, rows,
          std::chrono::duration_cast<utils::microseconds>(end - start));
}

template <size_t dim> struct fmt::formatter<math::ShapeArray<dim>> {
  constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const math::ShapeArray<dim>& p, FormatContext& ctx) const {
    return format_to(ctx.out(), "shape_array_t<{}>{}", dim,
                     fmt::join(p.data_, p.data_ + dim, ", "));
  }
};
template <std::size_t...Idxs>
constexpr auto substring_as_array(std::string_view str, std::index_sequence<Idxs...>)
{
  return std::array{str[Idxs]..., '\n'};
}

template <typename T>
constexpr auto type_name_array()
{
#if defined(__clang__)
  constexpr auto prefix   = std::string_view{"[T = "};
  constexpr auto suffix   = std::string_view{"]"};
  constexpr auto function = std::string_view{__PRETTY_FUNCTION__};
#elif defined(__GNUC__)
  constexpr auto prefix   = std::string_view{"with T = "};
  constexpr auto suffix   = std::string_view{"]"};
  constexpr auto function = std::string_view{__PRETTY_FUNCTION__};
#elif defined(_MSC_VER)
  constexpr auto prefix   = std::string_view{"type_name_array<"};
  constexpr auto suffix   = std::string_view{">(void)"};
  constexpr auto function = std::string_view{__FUNCSIG__};
#else
# error Unsupported compiler
#endif

  constexpr auto start = function.find(prefix) + prefix.size();
  constexpr auto end = function.rfind(suffix);

  static_assert(start < end);

  constexpr auto name = function.substr(start, (end - start));
  return substring_as_array(name, std::make_index_sequence<name.size()>{});
}

template <typename T>
struct type_name_holder {
  static inline constexpr auto value = type_name_array<T>();
};

template <typename T>
constexpr auto pretty_type_name() -> std::string_view
{
  constexpr auto& value = type_name_holder<T>::value;
  return std::string_view{value.data(), value.size()};
}

int main(int argc, char** argv) {
  init(argc, argv);
  // benchmark_mm(10);
  // benchmark_transpose_inplace(10);

  constexpr auto shape2 = math::make_shape(2, 3, 4);

  AX_INFO("Extent={}", shape2.Extent());
  AX_INFO("Stride={}", shape2.Stride());

  for (auto [i, j, k] : iter(shape2)) {
    AX_INFO("i: {}, j: {}, k: {} => {}", i, j, k, shape2(i, j, k));
    auto [ii, jj, kk] = shape2.Ind2Sub(shape2(i, j, k));
    AX_CHECK(i == ii && j == jj && k == kk, "Error sub2ind!{} {} {} => {}", ii, jj, kk,
             shape2(i, j, k));
    AX_CHECK(i < 3 && j < 4, "Error shape!");
  }

  math::RealFieldData field(shape2.Size());
  std::fill(field.begin(), field.end(), 1.0);

  auto u = make_accessor(std::as_const(field), shape2);
  for (auto [ijk, v] : enumerate(u)) {
    auto [i, j, k] = ijk;
    AX_INFO("i={}, j={}, k={}, v={}", i, j, k, v);
  }

  clean_up();
  return 0;
}
