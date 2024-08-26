#include <spdlog/fmt/std.h>

#include <ax/math/shape.hpp>

#include "ax/math/utils/formatting.hpp"
#include "ax/core/init.hpp"
#include "ax/core/logging.hpp"
#include "ax/math/accessor.hpp"
#include "ax/math/common.hpp"
#include "ax/math/utils/structure_binding.hpp"
#include "ax/math/views.hpp"
#include "ax/utils/pretty_typename.hpp"
#include "ax/utils/time.hpp"

#include <tbb/tbb.h>

using namespace ax;

static void benchmark_mm(int scale) {
  int rows = 1 << scale;
  math::Matrix<f32, math::dynamic, math::dynamic> m1(rows, rows);
  math::Matrix<f32, math::dynamic, math::dynamic> m2(rows, rows);
  m1.setRandom();
  m2.setRandom();

  auto start = utils::now();
  auto v = (m1 * m2).eval();
  auto end = utils::now();
  AX_INFO("Matrix multiplication: {} x {}, {}", rows, rows,
          std::chrono::duration_cast<utils::milliseconds>(end - start));
}

static void benchmark_transpose_inplace(int scale) {
  int rows = 1 << scale;
  math::Matrix<f32, math::dynamic, math::dynamic> m(rows, rows);
  m.setRandom();

  auto start = utils::now();
  m.transposeInPlace();
  auto end = utils::now();
  AX_INFO("Transpose inplace: {} x {}, {}", rows, rows,
          std::chrono::duration_cast<utils::microseconds>(end - start));
}

// template <typename IndexType, int dim>
// struct fmt::formatter<math::ShapeArray<IndexType, dim>> {
//   constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }
//
//   template <typename FormatContext>
//   auto format(const math::ShapeArray<IndexType, dim>& p, FormatContext& ctx) const {
//     return format_to(ctx.out(), "shape_array_t<{}>{}", dim,
//                      fmt::join(p.data_, p.data_ + dim, ", "));
//   }
// };
using namespace math;


int main(int argc, char** argv) {
  initialize(argc, argv);
  benchmark_mm(10);
  benchmark_transpose_inplace(10);

  // constexpr auto shape2 = make_shape(2, 3, 4);

  // AX_INFO("Extent={}", shape2.Extent());
  // AX_INFO("Stride={}", shape2.Stride());
  //
  // for (auto [i, j, k] : iter(shape2)) {
  //   AX_INFO("i: {}, j: {}, k: {} => {}", i, j, k, shape2(i, j, k));
  //   auto ijk = shape2.Ind2Sub(shape2(i, j, k));
  //   auto ii = ijk[0], jj = ijk[1], kk = ijk[2];
  //   AX_CHECK(i == ii && j == jj && k == kk, "Error sub2ind!{} {} {} => {}", ii, jj, kk,
  //            shape2(i, j, k));
  //   AX_CHECK(i < 3 && j < 4, "Error shape!");
  // }

  RealField3 f;
  f.resize(3, 4);
  for (auto [i, j] : iter(make_shape(3, 4))) {
    f(i, j) = i + j;
  }

  auto accessor = make_accessor(f, make_shape(4));
  for (auto [i, j] : iter(make_shape(3, 4))) {
    AX_CHECK(accessor(j)[i] == i + j, "Error accessor!");
  }

  for (auto ref : accessor) {
    std::cout << ref.transpose() << std::endl;
  }

  for (auto [ijk, v]: enumerate(accessor)) {
    std::cout << utils::pretty_type_name<decltype(ijk)>() << ": " << v.transpose() << std::endl;
  }

  for (auto v : accessor) {
    std::cout << v.transpose() << std::endl;
    v.setZero();
  }

  std::cout << f << std::endl;

  auto iter_shape = iter(accessor.GetShape());  // view

  auto shape34 = make_shape(3, 4);

  // AX_INFO("{}", *cursor);

  ShapeArray<size_t, 2> extent_end(3, 0);

  for(auto v: iter(shape34)) {
    std::cout << v[0] << v[1] << std::endl;
    AX_INFO("v={}", v);
  }


  auto [r, c] = extent_end;
  std::cout << utils::pretty_type_name<decltype(r)>() << std::endl;

  AX_INFO("{}, {} == {}", r, c, extent_end);

  RealVector3 d{3, 2, 1};
  auto [x, y, z] = unpack(RealVector3{3, 2, 1});
  AX_INFO("{}, {}, {}", x, y, z);
  AX_INFO("d={}", d);

  clean_up();
  return 0;
}
