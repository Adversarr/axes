// Reference:
//  CMU Convex Optimization 10-725/Proximal Gradient Descent

#include "ax/optim/optimizers/fista.hpp"

namespace ax::optim {

// fista method have a special linesearch.
OptResult Optimizer_Fista::Optimize(const OptProblem &problem, const math::vecxr &x0) const {
  // SECT: Checkings
  AX_THROW_IF_FALSE(problem.HasGrad(), "Gradient function not set");
  AX_THROW_IF_FALSE(problem.HasEnergy(), "Energy function not set");
  AX_THROW_IF_FALSE(problem.HasProximator(), "Proximator function not set");

  // SECT: Variables
  math::vecxr x = x0;
  math::vecxr grad = problem.EvalGrad(x), x_old = x;
  math::vecxr v = x;

  real theta = math::nan<real>, theta_old = 1, step_length_old = 0;
  real energy = problem.EvalEnergy(x);
  bool converged_grad, converged_var;
  idx iter;

  auto select_theta = [&iter, &theta_old, &step_length_old](real t) -> real {
    // t_(k-1) theta^2 = t theta_(k-1)^2 (1-theta), use positive root.
    // t_(k-1) theta^2 + t theta_(k-1)^2 theta - t theta_(k-1)^2 = 0
    if (iter == 0) {
      return 1;
    }
    real a = step_length_old;
    real b = t * theta_old * theta_old;
    real c = -b;

    real delta = b * b - 4 * a * c;
    real theta = (-b + std::sqrt(delta)) / (2 * a);
    return theta;
  };

  // SECT: Iteration
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
      AX_LOG(INFO) << "Fista iter " << iter << std::endl
                   << "  f: " << energy << std::endl
                   << "  grad_norm: " << grad.norm() << std::endl
                   << "  conv_grad: " << evalcg << std::endl
                   << "  conv_var: " << evalcv << std::endl
                   << "  theta: " << theta << std::endl;
    }
    if (converged_grad || converged_var) {
      break;
    }

    x_old = x;  // xk-1
    grad = problem.EvalGrad(x);

    // SECT: Fista Update
    real t = lr_;
    theta = select_theta(t);
    math::vecxr y = (1 - theta) * x_old + theta * v;
    x = y - t * problem.EvalGrad(y);
    x = problem.EvalProximator(x, t);
    auto converged_linesearch = [&]() -> bool {
      real energy_at_x = (energy = problem.EvalEnergy(x));
      real energy_at_y = problem.EvalEnergy(y);
      real grad_dot_xy = grad.dot(x - y);
      real displacement_norm = math::norm2(x - y);
      return energy_at_x <= energy_at_y + grad_dot_xy + 0.5 / t * displacement_norm;
    };

    while (!converged_linesearch()) {
      t *= shrink_rate_;
      theta = select_theta(t);
      y.noalias() = (1 - theta) * x_old + theta * v;
      x.noalias() = y - t * problem.EvalGrad(y);
      x = problem.EvalProximator(x, t);
    }

    // SECT: Update v, theta, step_length
    theta_old = theta;
    step_length_old = t;
    v.noalias() = 1 / theta * (x - x_old) + x_old;
  }

  OptResult result;
  result.x_opt_ = std::move(x);
  result.f_opt_ = energy;
  result.converged_var_ = converged_var;
  result.converged_grad_ = converged_grad;
  result.converged_ = converged_grad || converged_var;
  result.n_iter_ = iter;
  return result;
}

}  // namespace ax::optim
