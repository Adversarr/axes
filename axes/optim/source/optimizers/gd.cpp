#include "ax/optim/optimizers/gd.hpp"

#include "ax/core/excepts.hpp"
#include "ax/math/functional.hpp"
#include "ax/optim/common.hpp"

using namespace ax;
using namespace ax::optim;

OptResult GradientDescent::Optimize(OptProblem const& problem, math::vecxr const& x0) const {
  math::vecxr x = x0;
  idx n_dof = x.size();
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
    problem.EvalVerbose(iter, x, problem.EvalEnergy(x));
    // if (!math::isfinite(energy)) {
    //   return utils::FailedPreconditionError("Energy function returns Infinite number!");
    // }
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
    if (iter > 1 && enable_fista_) {
      real blend = real(iter - 2) / real(iter + 1);
      x = x + blend * (x - x_old);
    }
    x_old = x;
    grad = problem.EvalGrad(x);

    if (converged_grad || converged_var) {
      break;
    }

    math::vecxr dir = -grad;
    real step_length = lr_;
    if (linesearch_) {
      auto lsr = linesearch_->Optimize(problem, x, grad, dir);
      x = lsr.x_opt_;
      step_length = lsr.step_length_;
    } else {
      x += dir * lr_;
      step_length = lr_;
    }

    if (proximator_) {
      x = proximator_(x, step_length);
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

void ax::optim::GradientDescent::SetProximator(
    std::function<math::vecxr(math::vecxr const&, real)> proximator) {
  proximator_ = proximator;
}

void ax::optim::GradientDescent::SetLineSearch(UPtr<LinesearchBase> linesearch) {
  linesearch_ = std::move(linesearch);
}

void ax::optim::GradientDescent::SetLearningRate(real const& lr) { lr_ = lr; }

void GradientDescent::SetOptions(utils::Opt const& opt) {
  OptimizerBase::SetOptions(opt);
  AX_SYNC_OPT_IF(opt, real, lr) { AX_THROW_IF_LT(lr_, 0, "Learning Rate should be positive"); }
  AX_SYNC_OPT(opt, bool, enable_fista);
  auto [has_linesearch, linesearch] = utils::extract_enum<LineSearchKind>(opt, "linesearch");
  if (has_linesearch) {
    AX_THROW_IF_NULL(linesearch,
                     "Linesearch not found: " + std::string(opt.at("linesearch").as_string()));
    linesearch_ = LinesearchBase::Create(*linesearch);
    AX_THROW_IF_NULL(linesearch,
                     "Linesearch create failed: " + std::string(opt.at("linesearch").as_string()));
  }

  if (linesearch_) {
    utils::extract_tunable(opt, "linesearch_options", linesearch_.get());
  }
}

utils::Opt GradientDescent::GetOptions() const {
  auto opt = OptimizerBase::GetOptions();
  opt["lr"] = lr_;
  opt["enable_fista"] = enable_fista_;
  if (linesearch_) {
    auto name = utils::reflect_name(linesearch_->GetKind());
    opt["linesearch"] = name.value();
    opt["linesearch_options"] = linesearch_->GetOptions();
  }
  return opt;
}
