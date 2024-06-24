#include "ax/optim/optimizers/gd.hpp"

#include "ax/core/excepts.hpp"
#include "ax/math/functional.hpp"
#include "ax/optim/common.hpp"

using namespace ax;
using namespace ax::optim;

OptResult Optimizer_GradientDescent::Optimize(OptProblem const& problem,
                                              math::vecxr const& x0) const {
  math::vecxr x = x0;
  // if (!problem.HasGrad()) {
  //   return utils::FailedPreconditionError("Gradient function not set");
  // } else if (!problem.HasEnergy()) {
  //   return utils::FailedPreconditionError("Energy function not set");
  // }
  AX_THROW_IF_FALSE(problem.HasGrad(), "Gradient function not set");
  AX_THROW_IF_FALSE(problem.HasEnergy(), "Energy function not set");
  AX_THROW_IF_LT(lr_, 0, "Invalid learning rate: " + std::to_string(lr_));

  real energy = problem.EvalEnergy(x);
  math::vecxr grad, x_old = x;
  bool converged_grad = false;
  bool converged_var = false;
  idx iter = 0;
  for (iter = 0; iter < max_iter_; ++iter) {
    problem.EvalVerbose(iter, x, energy);
    AX_THROW_IF_FALSE(math::isfinite(energy), "Energy function returns Infinite number!");

    real evalcg = (iter > 0 && problem.HasConvergeGrad()) ? problem.EvalConvergeGrad(x, grad)
                                                          : math::inf<real>;
    real evalcv = (iter > 0 && problem.HasConvergeVar()) ? problem.EvalConvergeVar(x, x_old)
                                                         : math::inf<real>;
    converged_grad = evalcg < tol_grad_;
    converged_var = evalcv < tol_var_;
    if (verbose_) {
      AX_LOG(INFO) << "Gradient Descent iter " << iter << std::endl
                   << "  f: " << energy << std::endl
                   << "  grad_norm: " << grad.norm() << std::endl
                   << "  conv_grad: " << evalcg << std::endl
                   << "  conv_var: " << evalcv << std::endl;
    }

    x_old = x;
    grad = problem.EvalGrad(x);

    if (converged_grad || converged_var) {
      break;
    }

    math::vecxr const dir = -grad;
    if (linesearch_) {
      auto lsr = linesearch_->Optimize(problem, x, grad, dir);
      x = lsr.x_opt_;
    } else {
      x.noalias() += dir * lr_;
      if (problem.HasProximator()) {
        x = problem.EvalProximator(x, lr_);
      }
    }

    energy = problem.EvalEnergy(x);
  }

  OptResult result;
  result.converged_grad_ = converged_grad;
  result.converged_var_ = converged_var;
  result.n_iter_ = iter;
  result.x_opt_ = x;
  result.f_opt_ = energy;
  result.converged_ = converged_grad || converged_var;
  return result;
}

void ax::optim::Optimizer_GradientDescent::SetLineSearch(UPtr<LinesearchBase> linesearch) {
  linesearch_ = std::move(linesearch);
}

void ax::optim::Optimizer_GradientDescent::SetLearningRate(real const& lr) { lr_ = lr; }

void Optimizer_GradientDescent::SetOptions(utils::Options const& opt) {
  OptimizerBase::SetOptions(opt);
  AX_SYNC_OPT_IF(opt, real, lr) { 
    AX_THROW_IF_LT(lr_, 0, "Learning Rate should be positive");
  }
  utils::extract_and_create<LinesearchBase, LineSearchKind>(opt, "linesearch", linesearch_);
  utils::extract_tunable(opt, "linesearch_opt", linesearch_.get());
}

utils::Options Optimizer_GradientDescent::GetOptions() const {
  auto opt = OptimizerBase::GetOptions();
  opt["lr"] = lr_;
  if (linesearch_) {
    auto name = utils::reflect_name(linesearch_->GetKind());
    opt["linesearch"] = name.value();
    opt["linesearch_opt"] = linesearch_->GetOptions();
  }
  return opt;
}

OptimizerKind Optimizer_GradientDescent::GetKind() const {
  return OptimizerKind::kGradientDescent;
}

