#pragma once

#include "ax/optim/linesearch/linesearch.hpp"
#include "ax/optim/optimizer_base.hpp"

namespace ax::optim {

AX_DEFINE_ENUM_CLASS(NonlinearCgStrategy, kFletcherReeves, kPolakRibiere, kHestenesStiefel,
                     kDaiYuan, kPolakRibiereClamped, kHestenesStiefelClamped);

using NonlinearCgPreconditioner = std::function<Variable(const Variable&, const Gradient&)>;

namespace details {
NonlinearCgPreconditioner Dummy() noexcept;
}  // namespace details

class Optimizer_NonlinearCg final : public OptimizerBase {
public:
  Optimizer_NonlinearCg();

  ~Optimizer_NonlinearCg() override;

  OptimizerKind GetKind() const override;

  utils::Options GetOptions() const override;

  void SetStrategy(NonlinearCgStrategy strategy);

  void SetOptions(utils::Options const& option) override;

  OptResult Optimize(OptProblem const& prob, const Variable& x0) const override;

  void SetPreconditioner(NonlinearCgPreconditioner precond);

private:
  NonlinearCgStrategy strategy_ = NonlinearCgStrategy::kFletcherReeves;
  mutable NonlinearCgPreconditioner precond_;
  std::unique_ptr<LinesearchBase> linesearch_;
  Index restart_period_ = 50;
};
}  // namespace ax::optim
