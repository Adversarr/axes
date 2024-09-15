#pragma once
#include "common.hpp"
#include "ax/utils/opt.hpp"

namespace ax::math {

class BlockSolverBase : utils::Tunable {
public:
  BlockSolverBase() = default;
  virtual ~BlockSolverBase() = default;

  void SetProblem(std::unique_ptr<BlockedLinsysProblem> problem);

  void SetProblem(RealBlockMatrix A);

  // APIs
  virtual BlockedLinsysSolveStatus Solve(ConstRealBufferView b, RealBufferView x) const = 0;
  virtual void AnalyzePattern();
  virtual void Factorize();
  void Compute();
  virtual BlockSolverKind GetKind() const = 0;

  static std::unique_ptr<BlockSolverBase> Create(BlockSolverKind kind);

  std::shared_ptr<BlockedLinsysProblem> problem_;
  std::unique_ptr<BlockPreconditionerBase> preconditioner_;
};

}  // namespace ax::math