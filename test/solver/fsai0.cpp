#include <doctest/doctest.h>

#include "ax/math/sparse_matrix/linsys/preconditioner/fsai0.hpp"

// create a test case
// [3, 1, 0, 0]
// [1, 3, 1, 0]
// [0, 1, 3, 1]
// [0, 0, 1, 3]
// A stronger poission.
using namespace ax;
std::unique_ptr<math::RealCSRMatrix> create() {
  auto ptr = std::make_unique<math::RealCSRMatrix>(4, 4, BufferDevice::Host);

  math::RealSparseCOO coo;
  for (int i = 0; i < 4; ++i) {
    coo.emplace_back(i, i, 3);
    if (i > 0) {
      coo.emplace_back(i, i - 1, 1);
      coo.emplace_back(i - 1, i, 1);
    }
  }

  ptr->SetFromTriplets(coo);
  std::cout << ptr->ToSparseMatrix() << std::endl;
  return ptr;
}

TEST_CASE("fsai0 host") {
  math::RealSparseMatrixPtr csr = create();
  math::GeneralSparsePreconditioner_FSAI0 fsai0;

  fsai0.SetProblem(csr);

  fsai0.AnalyzePattern();

  fsai0.Factorize();

  auto approx_inv = fsai0.fact_inv_->ToSparseMatrix();
  std::cout << approx_inv << std::endl;
}