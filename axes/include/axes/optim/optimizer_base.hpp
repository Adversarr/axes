#pragma once

#include "axes/utils/opt.hpp"
#include "common.hpp"

namespace ax::optim {

class OptimizerBase {
public:
  static constexpr real default_tol_var = 1e-6;
  static constexpr real default_tol_grad = 1e-6;
  static constexpr idx default_max_iter = 100;

  /****************************** Ctor Dtor ******************************/
  explicit OptimizerBase(OptProblem& problem) : problem_{problem} {}

  virtual ~OptimizerBase() = default;

  /****************************** Interfaces ******************************/
  virtual OptResult Optimize(math::vecxr const& x0, utils::Opt const& options) = 0;

  /****************************** Getter Setter ******************************/
  void SetMaxIter(idx max_iter);

  void SetTolVar(real tol_var);

  void SetTolGrad(real tol_grad);

  inline OptProblem& GetProblem() { return problem_; }

  idx GetMaxIter() const;

  real GetTolVar() const;

  real GetTolGrad() const;

protected:
  OptProblem& problem_;

  /****************************** Options ******************************/
  idx max_iter_{default_max_iter};
  real tol_var_{default_tol_var};
  real tol_grad_{default_tol_grad};
};

}  // namespace ax::optim
