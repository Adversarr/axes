#include "ax/optim/optimizers/gd.hpp"

#include "ax/math/functional.hpp"
#include "ax/optim/common.hpp"

using namespace ax;
using namespace ax::optim;

OptResult GradientDescent::Optimize(OptProblem const& problem, math::vecxr const& x0) const {
  math::vecxr x = x0;
  idx n_dof = x.size();
  if (!problem.HasGrad()) {
    return utils::FailedPreconditionError("Gradient function not set");
  } else if (!problem.HasEnergy()) {
    return utils::FailedPreconditionError("Energy function not set");
  }
  real energy = problem.EvalEnergy(x);
  math::vecxr grad, x_old = x;
  bool converged_grad = false;
  bool converged_var = false;
  idx iter = 0;
  for (iter = 0; iter < max_iter_; ++iter) {
    problem.EvalVerbose(iter, x, problem.EvalEnergy(x));
    if (!math::isfinite(energy)) {
      return utils::FailedPreconditionError("Energy function returns Infinite number!");
    }

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
    if (enable_fista_) {
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
      if (!lsr.ok()) {
        return lsr.status();
      }
      x = lsr->x_opt_;
      step_length = lsr->step_length_;
    } else {
      x += dir * lr_;
      step_length = lr_;
    }

    if (proximator_) {
      x = proximator_(x, step_length);
    }

    energy = problem.EvalEnergy(x);
  }
  OptResultImpl result;
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

