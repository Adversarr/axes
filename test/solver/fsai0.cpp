#include "ax/math/sparse_matrix/linsys/preconditioner/fsai0.hpp"
#include <doctest/doctest.h>

// create a test case
// [3, 1, 0, 0]
// [1, 3, 1, 0]
// [0, 1, 3, 1]
// [0, 0, 1, 3]
// A stronger poission.
using namespace ax;
const int total = 10;

std::unique_ptr<math::RealCSRMatrix> create() {
  auto ptr = std::make_unique<math::RealCSRMatrix>(total, total, BufferDevice::Host);

  math::RealSparseCOO coo;
  for (int i = 0; i < total; ++i) {
    coo.emplace_back(i, i, 2.5);
    if (i > 0) {
      coo.emplace_back(i, i - 1, 1);
      coo.emplace_back(i - 1, i, 1);
    }
  }

  ptr->SetFromTriplets(coo);
  return ptr;
}

TEST_CASE("fsai0 host") {
  math::RealSparseMatrixPtr csr = create();
  math::GeneralSparsePreconditioner_FSAI0 fsai0;

  fsai0.SetProblem(csr);
  fsai0.AnalyzePattern();
  fsai0.Factorize();
  auto a = csr->ToSparseMatrix();

  auto approx_inv = fsai0.fact_inv_->ToSparseMatrix();
  math::RealSparseMatrix gagt = approx_inv * a * approx_inv.transpose();
  math::for_each_entry(gagt, [](Index i, Index j, Real v) {
    if (i == j) {
      CHECK(v == doctest::Approx(1.0));
    }
  });

  math::RealSparseMatrix ggt_a = approx_inv * approx_inv.transpose() * a;
  std::cout << ggt_a.toDense() << std::endl;
}
