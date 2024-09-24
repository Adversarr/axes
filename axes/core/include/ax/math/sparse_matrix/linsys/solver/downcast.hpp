#pragma once
#include "ax/math/linsys/sparse.hpp"
#include "ax/math/sparse_matrix/linsys/solver.hpp"

namespace ax::math {

class GeneralSparseSolver_Downcast final : public GeneralSparseSolverBase {
public:
  GeneralSparseSolver_Downcast();
  virtual ~GeneralSparseSolver_Downcast() = default;

  void AnalyzePattern() override;
  void Factorize() override;

  BlockedLinsysSolveStatus Solve(ConstRealBufferView b, RealBufferView x) const override;

  GeneralSparseSolverKind GetKind() const override {
    return GeneralSparseSolverKind::Downcast;
  }

protected:
  mutable RealVectorX host_b_;
  mutable RealVectorX host_x_;
  std::unique_ptr<HostSparseSolverBase> host_solver_;
};

}  // namespace ax::math