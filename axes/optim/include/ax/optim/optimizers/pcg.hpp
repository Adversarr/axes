#pragma once

#include "ax/optim/optimizer_base.hpp"

namespace ax::optim {

AX_DEFINE_ENUM_CLASS(NonlinearCgStrategy, FletcherReeves, PolakRibiere, HestenesStiefel, HagerZhang);

struct NonlinearCgPreconditioner {
  std::function<Variable(const Gradient&)> fn_;

  static NonlinearCgPreconditioner Dummy() noexcept {
    return { [](const Gradient& x) { return x; } };
  }
};

class Optimizer_NonlinearCg final: public OptimizerBase {
public:
  Optimizer_NonlinearCg();
  ~Optimizer_NonlinearCg() override;

  OptimizerKind GetKind() const override;
  utils::Options GetOptions() const override;
  void SetStrategy(NonlinearCgStrategy strategy);
  void SetOptions(utils::Options const& option) override;


  OptResult Optimize(OptProblem const& prob, const Variable& x0) const override;

private:
  NonlinearCgStrategy strategy_;
  NonlinearCgPreconditioner precond_;
};

}
