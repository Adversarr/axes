#pragma once
#include "common.hpp"

namespace ax::math {

class BlockPreconditionerBase : public utils::Tunable {
public:
  BlockPreconditionerBase() = default;
  virtual ~BlockPreconditionerBase() = default;

  virtual void Solve(ConstRealBufferView b, RealBufferView x) const = 0;
  virtual void AnalyzePattern() = 0;
  virtual void Factorize() = 0;

  virtual BlockPreconditionerKind GetKind() const = 0;

  void SetProblem(std::shared_ptr<BlockedLinsysProblem> problem);

  static std::unique_ptr<BlockPreconditionerBase> Create(BlockPreconditionerKind kind);

protected:
  std::shared_ptr<BlockedLinsysProblem> problem_;
};

}  // namespace ax::math