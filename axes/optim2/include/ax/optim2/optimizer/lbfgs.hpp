#pragma once
#include "base.hpp"

namespace ax::optim2 {

class Optimizer_LBFGS : public OptimizerBase {
public:
  using CentralFn = std::function<void(RealBufferView /* r */,
            ConstRealBufferView /* s */,
            ConstRealBufferView /* y */,
            bool /* is_sy_valid */)>;

  Optimizer_LBFGS();
  ~Optimizer_LBFGS() override;

  AX_NODISCARD OptimizeResult Optimize(OptimizeParam param) override;

  OptimizerKind GetKind() const override { return OptimizerKind::Lbfgs; }

  size_t history_size_ = 10;

  CentralFn precond_;

private:
  BufferPtr<Real> search_direction_;

  void PushHistory(size_t iter);
  void SolveApproximator(size_t iter, RealBufferView q);

  std::vector<BufferPtr<Real>> s_;
  std::vector<BufferPtr<Real>> y_;
  std::vector<Real> rho_;
  std::vector<Real> alpha_;
};

}  // namespace ax::optim2