#include "ax/core/buffer/eigen_support.hpp"
#include "ax/core/buffer/for_each.hpp"
#include "ax/core/buffer/host_buffer.hpp"
#include "ax/math/block_matrix/linsys/preconditioner/block_jacobi.hpp"
#include "ax/core/init.hpp"
#include "ax/math/utils/formatting.hpp"

using namespace ax;

math::RealBlockMatrix create_block_matrix() {
  // Allocate the buffer
  auto value = HostBuffer<Real>::Create({2, 2, 4}); // 2x2 block, 4 blocks
  auto row_ptr = HostBuffer<int>::Create({5}); // 4 rows +1 for the end
  auto col_indices = HostBuffer<int>::Create({4}); // 4 blocks
  // Fill the buffer
  auto [val, row, col] = make_view(value, row_ptr, col_indices);

  // Fill the values
  for (size_t bid = 0; bid < 4; ++bid) {
    for (size_t j = 0; j < 2; ++j) {
      for (size_t i = 0; i < 2; ++i) {
        val(i, j, bid) = i == j ? 3.0 : 1.0;
      }
    }
    row(bid) = static_cast<int>(bid);
    row(bid + 1) = static_cast<int>(bid + 1);
    col(bid) = static_cast<int>(bid);
  }

  math::RealBlockMatrix mat(4, 4, 2);
  mat.SetData(row, col, val);
  return mat;
}


static void test_block_jacobi() {
  math::BlockPreconditioner_BlockJacobi jac;
  auto prob = std::make_shared<math::BlockedLinsysProblem>(create_block_matrix());
  jac.SetProblem(prob);
  jac.AnalyzePattern();
  jac.Factorize();

  // ground truth for the block
  math::RealMatrix2 d;
  d << 3, 1, 1, 3;
  d = d.inverse().eval();
  AX_INFO("Test construct.");

  // check inv_diag_
  auto inv = jac.inv_diag_->View();
  for (size_t b = 0; b < 4; ++b) {
    for (size_t j = 0; j < 2; ++j) {
      for (size_t i = 0; i < 2; ++i) {
        AX_CHECK(inv(i, j, b) == d(i, j), "inv_diag_ is wrong, expect {:12.6e} got {:12.6e}",
                 d(i, j), inv(i, j, b));
      }
    }
  }

  math::RealField2 b(2, 4);
  b << 1, 2, 3, 4, 5, 6, 7, 8;

  AX_INFO("Test solve.");
  math::RealField2 x(2, 4), gt(2, 4);

  for (size_t i = 0; i < 4; ++i) {
    gt.col(i) = d * b.col(i);
  }

  jac.Solve(view_from_matrix(b), view_from_matrix(x));

  for (size_t i = 0; i < 4; ++i) {
    AX_CHECK((x.col(i) - gt.col(i)).norm() < 1e-9, "Solve is wrong, expect {} got {}",
             gt.col(i), x.col(i));
  }
  AX_INFO("Block Jacobi preconditioner test passed.");
}


int main(int argc, char** argv) {
  initialize(argc, argv);

  test_block_jacobi();
  clean_up();
  return 0;
}