#pragma once
#include "ax/utils/opt.hpp"
#include "common.hpp"

namespace ax::math {

class GeneralSparsePreconditionerBase : public utils::Tunable {
public:
  GeneralSparsePreconditionerBase() = default;
  virtual ~GeneralSparsePreconditionerBase() = default;

  virtual void Solve(ConstRealBufferView b, RealBufferView x) const = 0;
  virtual void AnalyzePattern() = 0;
  virtual void Factorize() = 0;

  virtual GeneralPreconditionerKind GetKind() const = 0;

  void SetProblem(RealSparseMatrixPtr mat);

  static std::unique_ptr<GeneralSparsePreconditionerBase> Create(GeneralPreconditionerKind kind);

protected:
  RealSparseMatrixPtr mat_;
};

}  // namespace ax::math