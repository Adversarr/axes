#include <benchmark/benchmark.h>
#include <openblas/cblas.h>

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

static void view_visit_foreach(benchmark::State& state) {
  const size_t N = state.range(0);
  using namespace ax;
  ax::math::RealField4 mat(4, N); // 4xN matrix
  for (auto _ : state) {
    // Do the visit
    auto view = ax::view_from_matrix(mat);
    for (auto& v : view) {
      v = sin(v) * cos(v) * tan(v); // make it a bit more complex
      auto dist = (&v - view.Data());
      benchmark::DoNotOptimize(v);
    }
    // then create a column view.
    auto col_view = ax::view_as_matrix_1d<Real, 4>(view);
    for (size_t i = 0 ; i < N; ++i) {
      auto col = col_view(i);
      for (size_t j = 0; j < 4; ++j) {
        Real& v = col(j);
        v = sin(v) * cos(v) * tan(v); // make it a bit more complex
        benchmark::DoNotOptimize(v);
      }
    }
  }
}

static void view_visit_naive_eigen(benchmark::State& state) {
  const size_t N = state.range(0);
  using namespace ax;
  Eigen::Matrix<Real, 4, Eigen::Dynamic> mat(4, N); // 4xN matrix
  for (auto _ : state) {
    // Do the visit
    for (Index i = 0; i < 4; ++i) {
      for (Index j = 0; j < N; ++j) {
        mat(i, j) = sin(mat(i, j)) * cos(mat(i, j)) * tan(mat(i, j)); // make it a bit more complex
        benchmark::DoNotOptimize(mat(i, j));
      }
    }

    // then create a column view.
    for (Index i = 0 ; i < N; ++i) {
      for (Index j = 0; j < 4; ++j) {
        mat(j, i) = sin(mat(j, i)) * cos(mat(j, i)) * tan(mat(j, i)); // make it a bit more complex
        benchmark::DoNotOptimize(mat(j, i));
      }
    }
  }
}

BENCHMARK(view_matmul_4);
BENCHMARK(view_matmul)->RangeMultiplier(2)->Range(1 << 2, 1 << 12);
BENCHMARK(blas_matmul)->RangeMultiplier(2)->Range(1 << 2, 1 << 12);
BENCHMARK(eigen_matmul)->RangeMultiplier(2)->Range(1 << 2, 1 << 12);

BENCHMARK(view_dot_prod)->RangeMultiplier(2)->Range(1 << 2, 1 << 12);
BENCHMARK(blas_dot_prod)->RangeMultiplier(2)->Range(1 << 2, 1 << 12);
BENCHMARK(eigen_dot_prod)->RangeMultiplier(2)->Range(1 << 2, 1 << 12);
BENCHMARK(view_visit_foreach)->RangeMultiplier(2)->Range(1 << 2, 1 << 12);
BENCHMARK(view_visit_naive_eigen)->RangeMultiplier(2)->Range(1 << 2, 1 << 12);

BENCHMARK_MAIN();