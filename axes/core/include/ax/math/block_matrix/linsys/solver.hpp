#include "common.hpp"

namespace ax::math {

class BlockSolverBase {
public:
  BlockSolverBase() = default;
  virtual ~BlockSolverBase() = default;

  void SetProblem(std::unique_ptr<BlockedLinsysProblem> problem) { problem_ = std::move(problem); }

  void SetProblem(RealBlockMatrix const& A);

  // APIs
  virtual void Solve(ConstRealBufferView b, RealBufferView x) const = 0;
  virtual void AnalyzePattern() = 0;
  virtual void Factorize() = 0;

private:
  std::unique_ptr<BlockedLinsysProblem> problem_;
  std::unique_ptr<BlockPreconditionerBase> preconditioner_;
};

}  // namespace ax::math