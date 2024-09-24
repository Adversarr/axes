#pragma once
#include "ax/utils/opt.hpp"
#include "preconditioner.hpp"
#include "common.hpp"

namespace ax::math {

class GeneralSparseSolverBase : utils::Tunable {
public:
  GeneralSparseSolverBase() = default;
  virtual ~GeneralSparseSolverBase() = default;

  void SetProblem(ConstRealSparseMatrixPtr mat);

  // APIs
  virtual BlockedLinsysSolveStatus Solve(ConstRealBufferView b, RealBufferView x) const = 0;
  virtual void AnalyzePattern();
  virtual void Factorize();
  void Compute();
  virtual GeneralSparseSolverKind GetKind() const = 0;

  static std::unique_ptr<GeneralSparseSolverBase> Create(GeneralSparseSolverKind kind);

  ConstRealSparseMatrixPtr mat_;  ///< The matrix to solve

  //////////////////// Iterative Solver ////////////////////

  size_t max_iteration_{1000};  ///< maximum number of iterations
  Real tolerance_{1e-14};       ///< stopping criterion
  std::unique_ptr<GeneralSparsePreconditionerBase> preconditioner_;
};

}  // namespace ax::math