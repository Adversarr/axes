#include "ax/math/block_matrix/block_matrix.hpp"
#include <doctest/doctest.h>
#include "ax/core/buffer/buffer_view.hpp"
#include "ax/core/buffer/eigen_support.hpp"
#include "ax/core/buffer/for_each.hpp"

using namespace ax;

TEST_CASE("block mm") {
  size_t block_size = 2;

  // Example from cuSPARSE
  Real values[12] = {1, 0, 2, 3, 4, 0, 0, 5, 6, 7, 8, 0};
  size_t row_ptrs[3] = {0, 2, 3};
  size_t col_indices[4] = {0, 2, 1};

  // Create block matrix
  math::RealBlockMatrix block_matrix(2, 3, 2);

  auto value_view = view_from_raw_buffer(values, Dim3{2, 2, 3});
  auto row_ptrs_view = view_from_raw_buffer(row_ptrs, 3);
  auto col_indices_view = view_from_raw_buffer(col_indices, 3);
  block_matrix.SetData(row_ptrs_view, col_indices_view, value_view);

  // Check block matrix
  auto sparse = block_matrix.ToSparseMatrix();
  CHECK(sparse.rows() == 4);
  CHECK(sparse.cols() == 6);

  for (size_t br = 0; br < 2; ++br) {
    size_t block_beg = row_ptrs[br];
    size_t block_end = row_ptrs[br + 1];
    for (size_t bc = block_beg; bc < block_end; ++bc) {
      size_t col = col_indices[bc];
      for (size_t i = 0; i < block_size; ++i) {
        for (size_t j = 0; j < block_size; ++j) {
          Index r = static_cast<Index>(br * block_size + i);
          Index c = static_cast<Index>(col * block_size + j);
          CHECK(sparse.coeff(r, c) == values[i + j * block_size + bc * block_size * block_size]);
        }
      }
    }
  }

  math::RealField2 rhs(2, 3);
  math::RealField2 dst(2, 2);
  auto rhs_view = view_from_matrix(rhs);
  auto dst_view = view_from_matrix(dst);
  math::RealVectorX ground_truth = math::RealVectorX::Random(6);
  for (size_t i = 0; i < 6; ++i) {
    rhs(i % 2, i / 2) = ground_truth(i);
  }
  ground_truth = sparse * ground_truth;
  block_matrix.RightMultiplyTo(rhs_view, dst_view);

  for (size_t i = 0; i < 2; ++i) {
    for (size_t j = 0; j < 2; ++j) {
      CHECK(dst(i, j) == doctest::Approx(ground_truth(i + j * 2)));
    }
  }
}