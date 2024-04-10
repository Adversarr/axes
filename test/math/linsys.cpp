#include <doctest/doctest.h>

#include <entt/core/hashed_string.hpp>
#include <entt/locator/locator.hpp>

#include "ax/math/linsys/dense.hpp"
#include "ax/math/linsys/sparse.hpp"

TEST_CASE("Refl") {
  using namespace ax::utils;
  using namespace ax::math;
  auto ldlt = reflect_enum<DenseSolverKind>("kLDLT");
  CHECK(ldlt.has_value());
  CHECK(ldlt.value() == DenseSolverKind::kLDLT);

  auto partialLU = reflect_enum<DenseSolverKind>("kPartialPivLU");
  CHECK(partialLU.has_value());
  CHECK(partialLU.value() == DenseSolverKind::kPartialPivLU);
}
using namespace ax::math;
DenseSolverKind kinds[] = {DenseSolverKind::kPartialPivLU,
                           DenseSolverKind::kFullPivLU,
                           DenseSolverKind::kJacobiSVD,
                           DenseSolverKind::kBDCSVD,
                           DenseSolverKind::kFullPivHouseHolderQR,
                           DenseSolverKind::kHouseholderQR,
                           DenseSolverKind::kColPivHouseholderQR,
                           DenseSolverKind::kCompleteOrthognalDecomposition};

TEST_CASE("Solve Invertible") {
  using namespace ax::math;
  matxxr A(2, 2);
  A << 1, 2, 3, 4;
  vecxr b = vecxr::Ones(2);
  vecxr x = A.inverse() * b;
  LinsysProblem_Dense A_b{A, b};
  for (auto kind : kinds) {
    auto solver = DenseSolverBase::Create(kind);
    CHECK(solver != nullptr);
    auto status = solver->Analyse(A_b);
    CHECK(status.ok());

    auto result = solver->Solve(b, {});
    CHECK(result.ok());
    CHECK(result.value().converged_);
    CHECK(result.value().solution_.isApprox(x));
  }
}

TEST_CASE("Solve Non-Invertible") {
  using namespace ax::math;
  matxxr A(2, 2);
  A << 1, 2, 2, 4;
  vecxr b = vecxr::Ones(2);
  LinsysProblem_Dense A_b{A, b};
  for (auto kind : {
           DenseSolverKind::kFullPivLU,
           DenseSolverKind::kFullPivHouseHolderQR,
           DenseSolverKind::kHouseholderQR,
           DenseSolverKind::kColPivHouseholderQR,
           DenseSolverKind::kCompleteOrthognalDecomposition,
       }) {
    auto solver = DenseSolverBase::Create(kind);
    CHECK(solver != nullptr);
    auto status = solver->Analyse(A_b);
    CHECK(!status.ok());
  }

  for (auto kind : {
           DenseSolverKind::kPartialPivLU,
           DenseSolverKind::kJacobiSVD,
           DenseSolverKind::kBDCSVD,
       }) {
    auto solver = DenseSolverBase::Create(kind);
    CHECK(solver != nullptr);
    auto status = solver->Analyse(A_b);
    CHECK(status.ok());

    auto result = solver->Solve(b, {});
    CHECK(result.ok());
  }
}

TEST_CASE("Sparse LU") {
  using namespace ax::math;
  sp_matxxr A(2, 2);
  A.insert(0, 0) = 3;
  A.insert(0, 1) = 1;
  A.insert(1, 0) = 1;
  A.insert(1, 1) = 3;
  A.makeCompressed();
  vecxr x = vecxr::Ones(2);
  vecxr b = A * x;
  LinsysProblem_Sparse A_b{A, b};
  for (auto kind : {SparseSolverKind::kLU, SparseSolverKind::kQR,
                    SparseSolverKind::kConjugateGradient, SparseSolverKind::kLDLT}) {
    auto solver = SparseSolverBase::Create(kind);
    CHECK(solver != nullptr);
    auto status = solver->Analyse(A_b);
    CHECK(status.ok());

    auto result = solver->Solve(b, {});
    CHECK(result.ok());
    CHECK(result.value().converged_);
    CHECK(result.value().solution_.isApprox(x));
  }
}
