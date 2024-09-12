#include <benchmark/benchmark.h>
#include <openblas64/cblas.h>

#include "ax/core/buffer/buffer_view.hpp"
#include "ax/core/buffer/eigen_support.hpp"
#include "ax/core/buffer/host_buffer.hpp"


static void view_matmul_4(benchmark::State& state) {
  using namespace ax;
  const size_t N = 4;
  auto buf = HostBuffer<Real>::Create({N, N});
  auto bufv = buf->View();
  auto matv = view_as_matrix_full<math::RealMatrix4>(bufv);

  auto buf2 = HostBuffer<Real>::Create({N, N});
  auto bufv2 = buf2->View();
  auto matv2 = view_as_matrix_full<math::RealMatrix4>(bufv2);

  auto buf3 = HostBuffer<Real>::Create({N, N});
  auto bufv3 = buf3->View();
  auto matv3 = view_as_matrix_full<math::RealMatrix4>(bufv3);
  for (auto _ : state) {

    matv3.noalias() = matv * matv2;
  }
}

static void view_matmul(benchmark::State& state) {
  using namespace ax;
  const size_t N = state.range(0);
  auto buf = HostBuffer<Real>::Create({N, N});
  auto bufv = buf->View();
  auto matv = view_as_matrix_full<math::RealMatrixX>(bufv);

  auto buf2 = HostBuffer<Real>::Create({N, N});
  auto bufv2 = buf2->View();
  auto matv2 = view_as_matrix_full<math::RealMatrixX>(bufv2);

  auto buf3 = HostBuffer<Real>::Create({N, N});
  auto bufv3 = buf3->View();
  auto matv3 = view_as_matrix_full<math::RealMatrixX>(bufv3);
  for (auto _ : state) {

    matv3.noalias() = matv * matv2;
  }
}

static void blas_matmul(benchmark::State& state) {
  using namespace ax;
  const size_t N = state.range(0);
  auto buf = HostBuffer<Real>::Create({N, N});
  auto bufv = buf->View();
  auto matv = view_as_matrix_full<math::RealMatrixX>(bufv);

  auto buf2 = HostBuffer<Real>::Create({N, N});
  auto bufv2 = buf2->View();
  auto matv2 = view_as_matrix_full<math::RealMatrixX>(bufv2);

  auto buf3 = HostBuffer<Real>::Create({N, N});
  auto bufv3 = buf3->View();
  auto matv3 = view_as_matrix_full<math::RealMatrixX>(bufv3);
  for (auto _ : state) {

    cblas_dgemm(CblasColMajor, CblasNoTrans, CblasNoTrans, N, N, N, 1.0, matv.data(), N,
                matv2.data(), N, 0.0, matv3.data(), N);
  }
}

static void eigen_matmul(benchmark::State& state) {
  using namespace ax;
  const size_t N = state.range(0);

  Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> mat(N, N);
  Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> mat2(N, N);
  Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> mat3(N, N);
  for (auto _ : state) {

    mat3.noalias() = mat * mat2;
  }
}

static void view_dot_prod(benchmark::State& state) {
  using namespace ax;
  const size_t N = state.range(0);
  auto buf = HostBuffer<Real>::Create({N, N});
  auto bufv = buf->View();
  auto vecv = view_as_matrix_full<math::RealMatrixX>(bufv);

  auto buf2 = HostBuffer<Real>::Create({N, N});
  auto bufv2 = buf2->View();
  auto vecv2 = view_as_matrix_full<math::RealMatrixX>(bufv2);
  for (auto _ : state) {

    auto dot = math::dot(vecv, vecv2);
    benchmark::DoNotOptimize(dot);
  }
}

static void blas_dot_prod(benchmark::State& state) {
  using namespace ax;
  const size_t N = state.range(0);
  auto buf = HostBuffer<Real>::Create({N, N});
  auto bufv = buf->View();
  auto vecv = view_as_matrix_full<math::RealMatrixX>(bufv);

  auto buf2 = HostBuffer<Real>::Create({N, N});
  auto bufv2 = buf2->View();
  auto vecv2 = view_as_matrix_full<math::RealMatrixX>(bufv2);
  for (auto _ : state) {
    auto dot = cblas_ddot(N * N, vecv.data(), 1, vecv2.data(), 1);
    benchmark::DoNotOptimize(dot);
  }
}

static void eigen_dot_prod(benchmark::State& state) {
  using namespace ax;
  const size_t N = state.range(0);
  Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> vec(N, N);
  Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> vec2(N, N);
  for (auto _ : state) {
    Real dot = math::dot(vec, vec2);
    benchmark::DoNotOptimize(dot);
  }
}

BENCHMARK(view_matmul_4);
BENCHMARK(view_matmul)->RangeMultiplier(2)->Range(1 << 2, 1 << 12);
BENCHMARK(blas_matmul)->RangeMultiplier(2)->Range(1 << 2, 1 << 12);
BENCHMARK(eigen_matmul)->RangeMultiplier(2)->Range(1 << 2, 1 << 12);

BENCHMARK(view_dot_prod)->RangeMultiplier(2)->Range(1 << 2, 1 << 12);
BENCHMARK(blas_dot_prod)->RangeMultiplier(2)->Range(1 << 2, 1 << 12);
BENCHMARK(eigen_dot_prod)->RangeMultiplier(2)->Range(1 << 2, 1 << 12);

BENCHMARK_MAIN();

// --------------------------------------------------------------
// Benchmark                    Time             CPU   Iterations
// --------------------------------------------------------------
// view_matmul_4             15.9 ns         15.9 ns     41972599
// view_matmul/4             42.5 ns         42.5 ns     16980812
// view_matmul/8              163 ns          163 ns      4379853
// view_matmul/16             601 ns          601 ns      1162535
// view_matmul/32            3856 ns         3853 ns       182083
// view_matmul/64            9021 ns         9013 ns        74379
// view_matmul/128         107455 ns       107205 ns         5969
// view_matmul/256         430510 ns       429427 ns         1685
// view_matmul/512        3398124 ns      3387366 ns          206
// view_matmul/1024      19730912 ns     19675590 ns           35
// view_matmul/2048     165997885 ns    162924618 ns            4
// view_matmul/4096    1171830740 ns   1157603560 ns            1
// blas_matmul/4             60.8 ns         60.8 ns     10835536
// blas_matmul/8              118 ns          118 ns      5567513
// blas_matmul/16             401 ns          400 ns      1747961
// blas_matmul/32            2130 ns         2129 ns       328882
// blas_matmul/64           12743 ns        12734 ns        54997
// blas_matmul/128          26731 ns        26705 ns        26297
// blas_matmul/256         149486 ns       148735 ns         4406
// blas_matmul/512         991961 ns       978267 ns          701
// blas_matmul/1024       7015406 ns      6847910 ns          108
// blas_matmul/2048      57003388 ns     56553478 ns           10
// blas_matmul/4096     444501175 ns    423276547 ns            2
// eigen_matmul/4            63.4 ns         63.4 ns      9338557
// eigen_matmul/8             203 ns          203 ns      3450194
// eigen_matmul/16            742 ns          742 ns       942869
// eigen_matmul/32           4699 ns         4695 ns       148446
// eigen_matmul/64           9495 ns         9487 ns        73144
// eigen_matmul/128        108630 ns       108385 ns         6045
// eigen_matmul/256        433055 ns       420507 ns         1698
// eigen_matmul/512       3344603 ns      3328425 ns          212
// eigen_matmul/1024     20707241 ns     20493726 ns           35
// eigen_matmul/2048    155628424 ns    152940489 ns            4
// eigen_matmul/4096   1155064535 ns   1132178973 ns            1
// view_dot_prod/4           2.26 ns         2.26 ns    232590767
// view_dot_prod/8           10.1 ns         10.1 ns     69895935
// view_dot_prod/16          43.1 ns         43.1 ns     16091529
// view_dot_prod/32           190 ns          190 ns      3689137
// view_dot_prod/64           770 ns          769 ns       910120
// view_dot_prod/128         3083 ns         3082 ns       227162
// view_dot_prod/256        12453 ns        12442 ns        56261
// view_dot_prod/512        49666 ns        49622 ns        14108
// view_dot_prod/1024      199464 ns       199274 ns         3520
// view_dot_prod/2048     1320378 ns      1318127 ns          528
// view_dot_prod/4096     5936850 ns      5926446 ns          112
// blas_dot_prod/4           7.52 ns         7.51 ns     93211600
// blas_dot_prod/8           10.3 ns         10.3 ns     67638231
// blas_dot_prod/16          27.4 ns         27.4 ns     25655392
// blas_dot_prod/32          98.4 ns         98.3 ns      7146439
// blas_dot_prod/64           521 ns          521 ns      1342692
// blas_dot_prod/128         1037 ns         1036 ns       600625
// blas_dot_prod/256         2264 ns         2262 ns       322584
// blas_dot_prod/512         7491 ns         7480 ns        76201
// blas_dot_prod/1024       39269 ns        39213 ns        13545
// blas_dot_prod/2048     1332724 ns      1254726 ns          619
// blas_dot_prod/4096     8329911 ns      7400686 ns          111
// eigen_dot_prod/4          2.02 ns         2.01 ns    344962067
// eigen_dot_prod/8          17.0 ns         17.0 ns     41150831
// eigen_dot_prod/16         61.1 ns         61.1 ns     11461328
// eigen_dot_prod/32          226 ns          226 ns      3095343
// eigen_dot_prod/64          825 ns          824 ns       849357
// eigen_dot_prod/128        3155 ns         3153 ns       221996
// eigen_dot_prod/256       12424 ns        12413 ns        56390
// eigen_dot_prod/512       49579 ns        49535 ns        14131
// eigen_dot_prod/1024     198035 ns       197849 ns         3528
// eigen_dot_prod/2048     790090 ns       789347 ns          880
// eigen_dot_prod/4096    3598138 ns      3594939 ns          194