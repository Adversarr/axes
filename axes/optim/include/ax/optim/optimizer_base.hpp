#pragma once

#include "ax/utils/opt.hpp"
#include "common.hpp"

namespace ax::optim {

class OptimizerBase : public utils::Tunable {
public:
  static constexpr real default_tol_var = 1e-6;
  static constexpr real default_tol_grad = 1e-6;
  static constexpr idx default_max_iter = 100;

  /****************************** Ctor Dtor ******************************/
  explicit OptimizerBase() = default;

  virtual void SetOptions(utils::Opt const& options) override;

  utils::Opt GetOptions() const override;

  virtual ~OptimizerBase() = default;

  /****************************** Interfaces ******************************/
  virtual OptResult Optimize(OptProblem const& problem, math::vecxr const& x0) const = 0;

  /****************************** Getter Setter ******************************/
  void SetMaxIter(idx max_iter);

  void SetTolVar(real tol_var);

  void SetTolGrad(real tol_grad);

  idx GetMaxIter() const;

  real GetTolVar() const;

  real GetTolGrad() const;

protected:
  /****************************** Options ******************************/
  idx max_iter_{default_max_iter};
  real tol_var_{default_tol_var};
  real tol_grad_{default_tol_grad};
  bool verbose_{false};
};

}  // namespace ax::optim
