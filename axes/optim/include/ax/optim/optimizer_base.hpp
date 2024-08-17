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
  static constexpr Index default_max_iter = 100;
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
  void SetMaxIter(Index max_iter);

  void SetTolVar(real tol_var);

  void SetTolGrad(real tol_grad);

  Index GetMaxIter() const;

  real GetTolVar() const;

  real GetTolGrad() const;

protected:
  // void RecordTrajectory(math::RealVectorX const& x, math::RealVectorX const& grad, real energy) const;
  // void ClearTrajectory() const;
  // mutable std::vector<math::RealVectorX> x_history_;
  // mutable std::vector<math::RealVectorX> grad_history_;
  // mutable std::vector<real> energy_history_;
  /****************************** Options ******************************/
  Index max_iter_{default_max_iter};
  real tol_var_{default_tol_var};
  real tol_grad_{default_tol_grad};
  bool verbose_{false};
  // bool record_trajectory_{false};
};

}  // namespace ax::optim
