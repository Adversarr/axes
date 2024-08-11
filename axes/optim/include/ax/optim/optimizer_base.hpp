#pragma once

#include "ax/utils/enum_refl.hpp"
#include "ax/utils/opt.hpp"
#include "common.hpp"

namespace ax::optim {

AX_DEFINE_ENUM_CLASS(OptimizerKind,
    kNewton,
    kGradientDescent,
    kLbfgs,
    kFista);

class OptimizerBase : public utils::Tunable {
public:
  static constexpr real default_tol_var = 1e-6;
  static constexpr real default_tol_grad = 1e-6;
  static constexpr idx default_max_iter = 100;
  static std::unique_ptr<OptimizerBase> Create(OptimizerKind k);

  /****************************** Ctor Dtor ******************************/
  explicit OptimizerBase() = default;

  void SetOptions(utils::Options const& options) override;

  utils::Options GetOptions() const override;

  virtual ~OptimizerBase() = default;

  /****************************** Interfaces ******************************/
  virtual OptResult Optimize(OptProblem const& problem, Variable const& x) const = 0;

  virtual OptimizerKind GetKind() const = 0;

  /****************************** Getter Setter ******************************/
  void SetMaxIter(idx max_iter);

  void SetTolVar(real tol_var);

  void SetTolGrad(real tol_grad);

  idx GetMaxIter() const;

  real GetTolVar() const;

  real GetTolGrad() const;

protected:
  // void RecordTrajectory(math::vecxr const& x, math::vecxr const& grad, real energy) const;
  // void ClearTrajectory() const;
  // mutable std::vector<math::vecxr> x_history_;
  // mutable std::vector<math::vecxr> grad_history_;
  // mutable std::vector<real> energy_history_;
  /****************************** Options ******************************/
  idx max_iter_{default_max_iter};
  real tol_var_{default_tol_var};
  real tol_grad_{default_tol_grad};
  bool verbose_{false};
  // bool record_trajectory_{false};
};

}  // namespace ax::optim
