#pragma once
#include "ax/utils/opt.hpp"
#include "common.hpp"

namespace ax::math {

/****************************** Embedded Solver Kinds ******************************/
BOOST_DEFINE_FIXED_ENUM_CLASS(DenseSolverKind, Index,
  // SPSD
  kLDLT,
  kLLT,

  // LU Solvers
  kPartialPivLU,
  kFullPivLU,

  // QR-based Solvers
  kHouseholderQR,
  kColPivHouseholderQR,
  kFullPivHouseHolderQR,
  kCompleteOrthognalDecomposition,

  // SVD-based Solvers
  kJacobiSVD,
  kBDCSVD);

/****************************** Implement ******************************/
class DenseSolverBase : public utils::Tunable {
public:
  static std::unique_ptr<DenseSolverBase> Create(DenseSolverKind kind);

  virtual ~DenseSolverBase() = default;

  DenseSolverBase& SetProblem(std::shared_ptr<LinsysProblem_Dense> problem) {
    cached_problem_ = std::move(problem);
    return *this;
  }

  DenseSolverBase& SetProblem(matxxr const &A) {
    return SetProblem(make_dense_problem(A));
  }

  DenseSolverBase& SetProblem(matxxr &&A) {
    return SetProblem(make_dense_problem(A));
  }

  virtual void Compute() = 0;
  virtual RealVectorX Solve(RealVectorX const &b) = 0;
  virtual DenseSolverKind GetKind() const = 0;


protected:
  std::shared_ptr<LinsysProblem_Dense> cached_problem_;
};

}  // namespace ax::math

