#include "ax/core/init.hpp"
#include "ax/math/common.hpp"
#include <chrono>

using namespace ax;
using namespace math;

double launch_test_abc(idx n, idx test_iteration) {
  vecxr a, b, c, d;
  // Test a * b + c.
  a.setRandom(n);
  b.setRandom(n);
  c.setRandom(n);
  d.setRandom(n);

  auto start_time = std::chrono::high_resolution_clock::now();
  for (idx i = 0; i < test_iteration; i++) {
    d.noalias() = a * b + c;
  }
  auto end_time = std::chrono::high_resolution_clock::now();
  return (end_time - start_time).count() / (real) test_iteration;
}

int main(int argc, char* argv[]) {
  ax::init(argc, argv);
  math::vecxr gflops_list(26-4);
  for (idx i = 4; i < 26; i++) {
    idx n = 1 << i;
    idx test_iteration = 512;
    double time = launch_test_abc(n, test_iteration);
    printf("i = %ld, n = %ld, time = %lf\n", i, n, time);
    // how many operations?
    // a * b + c = 1 mul + 1 add, n times.
    // total operations = 2 * n, then we need the FLOPs
    double flops = 2 * n / time;
    double gflops = flops / 1e9;
    printf("GFlops = %lf\n", gflops);
    gflops_list(i-4) = gflops;
  }
  printf("Average: %lf", (real) gflops_list.mean());
  ax::clean_up();
  return 0;
}