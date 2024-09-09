#include <doctest/doctest.h>

#include <entt/core/hashed_string.hpp>
#include <entt/locator/locator.hpp>

#include "ax/math/linsys/dense.hpp"
#include "ax/math/linsys/sparse.hpp"

TEST_CASE("Refl") {
  using namespace ax::utils;
  using namespace ax::math;
  auto ldlt = reflect_enum<DenseSolverKind>("LDLT");
  CHECK(ldlt.has_value());
  CHECK(ldlt.value() == DenseSolverKind::LDLT);

  auto partialLU = reflect_enum<DenseSolverKind>("PartialPivLU");
  CHECK(partialLU.has_value());
  CHECK(partialLU.value() == DenseSolverKind::PartialPivLU);
}
using namespace ax::math;
DenseSolverKind kinds[] = {DenseSolverKind::PartialPivLU,
                           DenseSolverKind::FullPivLU,
                           DenseSolverKind::JacobiSVD,
                           DenseSolverKind::BDCSVD,
                           DenseSolverKind::FullPivHouseHolderQR,
                           DenseSolverKind::HouseholderQR,
                           DenseSolverKind::ColPivHouseholderQR,
                           DenseSolverKind::CompleteOrthognalDecomposition};

TEST_CASE("Solve Invertible") {
  using namespace ax::math;
  RealMatrixX A(2, 2);
  A << 1, 2, 3, 4;
  RealVectorX b = RealVectorX::Ones(2);
  RealVectorX x = A.inverse() * b;
  for (auto kind : kinds) {
    auto solver = DenseSolverBase::Create(kind);
    CHECK(solver != nullptr);
    solver->SetProblem(A).Compute();
    auto result = solver->Solve(b);
    CHECK(result.isApprox(x));
  }
}

TEST_CASE("Solve Non-Invertible") {
  using namespace ax::math;
  RealMatrixX A(2, 2);
  A << 1, 2, 2, 4;
  RealVectorX b = RealVectorX::Ones(2);
  for (auto kind : {
           DenseSolverKind::FullPivLU,
           DenseSolverKind::FullPivHouseHolderQR,
           DenseSolverKind::HouseholderQR,
           DenseSolverKind::ColPivHouseholderQR,
           DenseSolverKind::CompleteOrthognalDecomposition,
       }) {
    auto solver = DenseSolverBase::Create(kind);
    CHECK(solver != nullptr);
    try {
      solver->SetProblem(A).Compute();
      CHECK(false);
    } catch (std::exception const &e) {
    }
  }

  for (auto kind : {
           DenseSolverKind::PartialPivLU,
           DenseSolverKind::JacobiSVD,
           DenseSolverKind::BDCSVD,
       }) {
    auto solver = DenseSolverBase::Create(kind);
    CHECK(solver != nullptr);
    try {
      solver->SetProblem(A).Compute();
      auto result = solver->Solve(b);
    } catch (std::exception const &e) {
      INFO(ax::utils::reflect_name(kind).value_or("Unknown"));
      INFO(e.what());
      CHECK(false);
    }
  }
}

TEST_CASE("Sparse LU") {
  using namespace ax::math;
  RealSparseMatrix A(2, 2);
  A.insert(0, 0) = 4;
  A.insert(0, 1) = 1;
  A.insert(1, 0) = 1;
  A.insert(1, 1) = 4;
  A.makeCompressed();
  RealMatrixX x = RealVectorX::Ones(2);
  RealMatrixX b = A * x;
  for (auto kind : {SparseSolverKind::LU, SparseSolverKind::QR,
                    SparseSolverKind::ConjugateGradient, SparseSolverKind::LDLT,
                    SparseSolverKind::Cholmod}) {
    auto solver = SparseSolverBase::Create(kind);
    CHECK(solver != nullptr);
    solver->SetProblem(A).Compute();
    auto result = solver->Solve(b, {});
    CHECK(result.solution_.isApprox(x));
  }
}
