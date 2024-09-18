#include <fmt/chrono.h>

#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/create_buffer.hpp"
#include "ax/core/buffer/eigen_support.hpp"
#include "ax/core/init.hpp"
#include "ax/math/sparse_matrix/block_matrix.hpp"
#include "ax/math/sparse_matrix/csr.hpp"
#include "ax/utils/time.hpp"
using namespace ax;

size_t prob_n = 128;

static void test_spmv_cpu() {
  // Allocate a CSR matrix from a poission problem 5 point stencil
  math::RealSparseCOO coo;

  for (size_t i = 0; i < prob_n; ++i) {
    for (size_t j = 0; j < prob_n; ++j) {
      if (i > 0) {
        coo.emplace_back(i * prob_n + j, (i - 1) * prob_n + j, -1);
      }
      if (j > 0) {
        coo.emplace_back(i * prob_n + j, i * prob_n + j - 1, -1);
      }
      coo.emplace_back(i * prob_n + j, i * prob_n + j, 4);
      if (j < prob_n - 1) {
        coo.emplace_back(i * prob_n + j, i * prob_n + j + 1, -1);
      }
      if (i < prob_n - 1) {
        coo.emplace_back(i * prob_n + j, (i + 1) * prob_n + j, -1);
      }
    }
  }
  size_t rows = prob_n * prob_n;
  math::RealCSRMatrix csr(rows, rows, BufferDevice::Host);
  csr.SetFromTriplets(coo);
  csr.Finish();

  // Allocate vectors
  math::RealMatrixX x = math::RealMatrixX::Random(rows, 3);
  math::RealMatrixX y = math::RealMatrixX::Random(rows, 3);
  math::RealMatrixX x_copy = x, y_copy = y;
  auto start = utils::now();
  csr.Multiply(view_from_matrix(x), view_from_matrix(y), 0.3, 0.7);  // y = .7 y + .3 A x
  auto end = utils::now();
  AX_INFO("CPU time: {}", end - start);

  // Compare with Eigen
  auto eigen = math::make_sparse_matrix(rows, rows, coo);
  y_copy = (0.7 * y_copy + 0.3 * eigen * x_copy).eval();

  for (size_t i = 0; i < rows; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      if (std::abs(y(i, j) - y_copy(i, j)) > 1e-6) {
        AX_THROW_RUNTIME_ERROR("{} {} mismatch: {} v.s. {}", i, j, y(i, j), y_copy(i, j));
      }
    }
  }
}

static void test_spmv_gpu() {
  // Allocate a CSR matrix from a poission problem 5 point stencil
  math::RealSparseCOO coo;

  for (size_t i = 0; i < prob_n; ++i) {
    for (size_t j = 0; j < prob_n; ++j) {
      if (i > 0) {
        coo.emplace_back(i * prob_n + j, (i - 1) * prob_n + j, -1);
      }
      if (j > 0) {
        coo.emplace_back(i * prob_n + j, i * prob_n + j - 1, -1);
      }
      coo.emplace_back(i * prob_n + j, i * prob_n + j, 4);
      if (j < prob_n - 1) {
        coo.emplace_back(i * prob_n + j, i * prob_n + j + 1, -1);
      }
      if (i < prob_n - 1) {
        coo.emplace_back(i * prob_n + j, (i + 1) * prob_n + j, -1);
      }
    }
  }
  size_t rows = prob_n * prob_n;
  math::RealCSRMatrix csr(rows, rows, BufferDevice::Device);
  csr.SetFromTriplets(coo);
  csr.Finish();

  // Allocate vectors
  math::RealVectorX x = math::RealVectorX::Random(rows);
  math::RealVectorX y = math::RealVectorX::Random(rows);
  math::RealVectorX x_copy = x, y_copy = y;

  auto x_d = create_buffer<Real>(BufferDevice::Device, {rows});
  auto y_d = create_buffer<Real>(BufferDevice::Device, {rows});
  copy(x_d->View(), view_from_matrix(x));
  copy(y_d->View(), view_from_matrix(y));
  auto start = utils::now();
  csr.Multiply(x_d->ConstView(), y_d->View(), 0.3, 0.7);  // y = .7 y + .3 A x
  cudaDeviceSynchronize();
  auto end = utils::now();
  copy(view_from_matrix(y), y_d->ConstView());

  AX_INFO("GPU time: {}", end - start);

  // Compare with Eigen
  auto eigen = math::make_sparse_matrix(rows, rows, coo);
  y_copy = (0.7 * y_copy + 0.3 * eigen * x_copy).eval();

  for (size_t i = 0; i < rows; ++i) {
    if (std::abs(y(i) - y_copy(i)) > 1e-6) {
      AX_THROW_RUNTIME_ERROR("y({}) = {}, y_copy({}) = {}", i, y(i), i, y_copy(i));
    }
  }
}

static void test_bsr_as_csr_cpu() {
  // Allocate a CSR matrix from a poission problem 5 point stencil
  math::RealSparseCOO coo;

  for (size_t i = 0; i < prob_n; ++i) {
    for (size_t j = 0; j < prob_n; ++j) {
      if (i > 0) {
        coo.emplace_back(i * prob_n + j, (i - 1) * prob_n + j, -1);
      }
      if (j > 0) {
        coo.emplace_back(i * prob_n + j, i * prob_n + j - 1, -1);
      }
      coo.emplace_back(i * prob_n + j, i * prob_n + j, 4);
      if (j < prob_n - 1) {
        coo.emplace_back(i * prob_n + j, i * prob_n + j + 1, -1);
      }
      if (i < prob_n - 1) {
        coo.emplace_back(i * prob_n + j, (i + 1) * prob_n + j, -1);
      }
    }
  }
  size_t rows = prob_n * prob_n;

  math::RealBlockMatrix csr(rows, rows, 1, BufferDevice::Host);
  {
    std::vector<int> rows, cols;
    std::vector<Real> v;
    for (const auto& triplet : coo) {
      rows.push_back(triplet.row());
      cols.push_back(triplet.col());
      v.push_back(triplet.value());
    }

    csr.SetFromBlockedTriplets(view_from_buffer(rows), view_from_buffer(cols),
                               view_from_buffer(v, {1, 1, v.size()}));
  }
  csr.Finish();

  // Allocate vectors
  math::RealMatrixX x = math::RealMatrixX::Random(rows, 1);
  math::RealMatrixX y = math::RealMatrixX::Random(rows, 1);
  math::RealMatrixX x_copy = x, y_copy = y;
  auto start = utils::now();
  csr.Multiply(view_from_raw_buffer(x.data(), {1, rows}), view_from_raw_buffer(y.data(), {1, rows}),
               0.3,
               0.7);  // y = .7 y + .3 A x
  auto end = utils::now();
  AX_INFO("CPU time: {}", end - start);

  // Compare with Eigen
  auto eigen = math::make_sparse_matrix(rows, rows, coo);
  y_copy = (0.7 * y_copy + 0.3 * eigen * x_copy).eval();

  for (size_t i = 0; i < rows; ++i) {
    for (size_t j = 0; j < 1; ++j) {
      if (std::abs(y(i, j) - y_copy(i, j)) > 1e-6) {
        AX_THROW_RUNTIME_ERROR("{} {} mismatch: {} v.s. {}", i, j, y(i, j), y_copy(i, j));
      }
    }
  }
}


static void test_bsr_as_csr_gpu() {
  // Allocate a CSR matrix from a poission problem 5 point stencil
  math::RealSparseCOO coo;

  for (size_t i = 0; i < prob_n; ++i) {
    for (size_t j = 0; j < prob_n; ++j) {
      if (i > 0) {
        coo.emplace_back(i * prob_n + j, (i - 1) * prob_n + j, -1);
      }
      if (j > 0) {
        coo.emplace_back(i * prob_n + j, i * prob_n + j - 1, -1);
      }
      coo.emplace_back(i * prob_n + j, i * prob_n + j, 4);
      if (j < prob_n - 1) {
        coo.emplace_back(i * prob_n + j, i * prob_n + j + 1, -1);
      }
      if (i < prob_n - 1) {
        coo.emplace_back(i * prob_n + j, (i + 1) * prob_n + j, -1);
      }
    }
  }
  size_t rows = prob_n * prob_n;

  math::RealBlockMatrix csr(rows, rows, 1, BufferDevice::Device);
  {
    std::vector<int> rows, cols;
    std::vector<Real> v;
    for (const auto& triplet : coo) {
      rows.push_back(triplet.row());
      cols.push_back(triplet.col());
      v.push_back(triplet.value());
    }

    csr.SetFromBlockedTriplets(view_from_buffer(rows), view_from_buffer(cols),
                               view_from_buffer(v, {1, 1, v.size()}));
  }
  csr.Finish();

  // Allocate vectors
  math::RealVectorX x = math::RealVectorX::Random(rows, 1);
  math::RealVectorX y = math::RealVectorX::Random(rows, 1);
  math::RealVectorX x_copy = x, y_copy = y;

  auto x_d = create_buffer<Real>(BufferDevice::Device, {1, rows});
  auto y_d = create_buffer<Real>(BufferDevice::Device, {1, rows});
  copy(x_d->View(), view_from_matrix(x).Reshaped({1, rows}));
  copy(y_d->View(), view_from_matrix(y).Reshaped({1, rows}));
  auto start = utils::now();
  csr.Multiply(x_d->ConstView(), y_d->View(), 0.3, 0.7);  // y = .7 y + .3 A x
  cudaDeviceSynchronize();
  auto end = utils::now();
  copy(view_from_matrix(y), y_d->ConstView());

  AX_INFO("GPU time: {}", end - start);

  // Compare with Eigen
  auto eigen = math::make_sparse_matrix(rows, rows, coo);
  y_copy = (0.7 * y_copy + 0.3 * eigen * x_copy).eval();

  for (size_t i = 0; i < rows; ++i) {
    if (std::abs(y(i) - y_copy(i)) > 1e-6) {
      AX_THROW_RUNTIME_ERROR("y({}) = {}, y_copy({}) = {}", i, y(i), i, y_copy(i));
    }
  }
}

#define EXECUTE_TEST(test)     \
  {                            \
    AX_INFO("Running " #test); \
    test();                    \
    AX_INFO("Passed " #test);  \
  }

int main(int argc, char** argv) {
  initialize(argc, argv);
  EXECUTE_TEST(test_spmv_cpu);
  EXECUTE_TEST(test_spmv_gpu);
  EXECUTE_TEST(test_bsr_as_csr_cpu);
  EXECUTE_TEST(test_bsr_as_csr_gpu);
  clean_up();
  return 0;
}