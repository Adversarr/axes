#include <benchmark/benchmark.h>
#include <benchmark/export.h>
#include <vecLib/cblas.h>
#include <vecLib/vecLib.h>

#include <Eigen/Eigen>

Eigen::Matrix<float, 3, 3> a, b;
void eigen_mm(benchmark::State& state) {
  a.setRandom();
  b.setRandom();
  for (auto s : state) {
    benchmark::DoNotOptimize((a * b).eval());
  }
}

void blas_mm(benchmark::State& state) {
  a.setRandom();
  b.setRandom();
  for (auto s : state) {
    float c[9];
  }
}

BENCHMARK(blas_mm);
BENCHMARK(eigen_mm);
