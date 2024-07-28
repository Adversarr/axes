#include <spdlog/fmt/std.h>

#include "ax/core/init.hpp"
#include "ax/core/logging.hpp"
#include "ax/math/common.hpp"
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

int main(int argc, char** argv) {
  ax::init(argc, argv);
  benchmark_mm(10);
  benchmark_transpose_inplace(10);
  ax::clean_up();
  return 0;
}
