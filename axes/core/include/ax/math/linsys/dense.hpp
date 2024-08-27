#pragma once
#include "ax/utils/opt.hpp"
#include "common.hpp"

namespace ax::math {

/****************************** Embedded Solver Kinds ******************************/
AX_DEFINE_ENUM_CLASS(DenseSolverKind,
                     // SPSD
                     LDLT, LLT,

                     // LU Solvers
                     PartialPivLU, FullPivLU,

                     // QR-based Solvers
                     HouseholderQR, ColPivHouseholderQR, FullPivHouseHolderQR,
                     CompleteOrthognalDecomposition,

                     // SVD-based Solvers
                     JacobiSVD, BDCSVD);

/****************************** Implement ******************************/
class DenseSolverBase : public utils::Tunable {
public:
  static std::unique_ptr<DenseSolverBase> Create(DenseSolverKind kind);

  ~DenseSolverBase() override = default;

  DenseSolverBase& SetProblem(std::shared_ptr<LinsysProblem_Dense> problem) {
    cached_problem_ = std::move(problem);
    return *this;
  }

  DenseSolverBase& SetProblem(RealMatrixX const& A) {
    return SetProblem(make_dense_problem(A));
  }

  DenseSolverBase& SetProblem(RealMatrixX&& A) {
    return SetProblem(make_dense_problem(A));
  }

  virtual void Compute() = 0;
  virtual RealVectorX Solve(RealVectorX const& b) = 0;
  virtual DenseSolverKind GetKind() const = 0;

protected:
  std::shared_ptr<LinsysProblem_Dense> cached_problem_;
};

}  // namespace ax::math
