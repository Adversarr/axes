#pragma once
#include "base.hpp"

namespace ax::optim2 {

AX_DEFINE_ENUM_CLASS(NonlinearCgStrategy, FletcherReeves, PolakRibiere, HestenesStiefel, DaiYuan,
                     PolakRibiereClamped, HestenesStiefelClamped);

/**
 * @brief Implements the Nonlinear Conjugate Gradient method. (with preconditioning)
 *
 * @note Refer to: "Novel preconditioners based on quasi–Newton updates for nonlinear conjugate
 * gradient methods" Caliciotti Andrea, Fasano Giovanni, Roma Massimo. Optim Lett (2017)
 */
class Optimizer_NonlinearCg final : public OptimizerBase {
public:
  Optimizer_NonlinearCg();

  ~Optimizer_NonlinearCg() override;

  OptimizerKind GetKind() const override;

  OptimizeResult Optimize(OptimizeParam param) override;

  NonlinearCgStrategy strategy_ = NonlinearCgStrategy::FletcherReeves;

  std::function<void(RealBufferView)> precond_;

private:
  BufferPtr<Real> prec_grad_;
  BufferPtr<Real> search_direction_;
  size_t restart_period_ = 50;
};

}  // namespace ax::optim2