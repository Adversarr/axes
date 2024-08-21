// Reference:
//  CMU Convex Optimization 10-725/Proximal Gradient Descent

#include "ax/optim/optimizers/fista.hpp"

#include "ax/core/logging.hpp"

namespace ax::optim {

// fista method have a special linesearch.
OptResult Optimizer_Fista::Optimize(const OptProblem &problem, const Variable &x0) const {
  // SECT: Checkings
  AX_THROW_IF_FALSE(problem.HasGrad(), "Gradient function not set");
  AX_THROW_IF_FALSE(problem.HasEnergy(), "Energy function not set");
  AX_THROW_IF_FALSE(problem.HasProximator(), "Proximator function not set");

  // SECT: Variables
  Variable x = x0;
  Gradient grad = problem.EvalGrad(x), x_old = x;
  Variable v = x;

  Real theta_old = 1, step_length_old = 0;
  Real energy = problem.EvalEnergy(x);
  bool converged_grad = false, converged_var = false;
  Index iter;

  auto select_theta = [&iter, &theta_old, &step_length_old](Real t) -> Real {
    // t_(k-1) theta^2 = t theta_(k-1)^2 (1-theta), use positive root.
    // t_(k-1) theta^2 + t theta_(k-1)^2 theta - t theta_(k-1)^2 = 0
    if (iter == 0) {
      return 1;
    }
    Real a = step_length_old;
    Real b = t * theta_old * theta_old;
    Real c = -b;

    Real delta = b * b - 4 * a * c;
    Real theta = (-b + std::sqrt(delta)) / (2 * a);
    return theta;
  };

  // SECT: Iteration
  for (iter = 0; iter < max_iter_; ++iter) {
    problem.EvalVerbose(iter, x, energy);
    AX_THROW_IF_FALSE(math::isfinite(energy), "Energy function returns Infinite number!");

    Real evalcg = (iter > 0 && problem.HasConvergeGrad()) ? problem.EvalConvergeGrad(x, grad)
                                                          : math::inf<Real>;
    Real evalcv = (iter > 0 && problem.HasConvergeVar()) ? problem.EvalConvergeVar(x, x_old)
                                                         : math::inf<Real>;
    converged_grad = evalcg < tol_grad_;
    converged_var = evalcv < tol_var_;

    if (iter >= 1) {
      converged_var |= (x - x_old).norm() < tol_var_ * step_length_old;
    }
    if (verbose_) {
      AX_INFO(
          "Fista iter {}:\n"
          "  f: {}\n"
          "  grad_norm: {}\n"
          "  conv_grad: {}\n"
          "  conv_var: {}\n"
          "  theta: {}",
          iter, energy, grad.norm(), evalcg, evalcv, theta_old);
    }

    if (converged_grad || converged_var) {
      break;
    }

    x_old = x;  // xk-1
    grad = problem.EvalGrad(x);

    Variable y = x;
    Real t = lr_;
    auto converged_linesearch = [&]() -> bool {
      Real energy_at_x = (energy = problem.EvalEnergy(x));
      Real energy_at_y = problem.EvalEnergy(y);
      Real grad_dot_xy = math::dot(problem.EvalGrad(y), x - y);
      Real displacement_norm = math::norm2(x - y);
      return energy_at_x <= energy_at_y + grad_dot_xy + 0.5 / t * displacement_norm;
    };
    // SECT: Fista Update
    Real theta = select_theta(t);
    y = (1 - theta) * x_old + theta * v;
    x = y - t * problem.EvalGrad(y);
    x = problem.EvalProximator(x, t);

    while (!converged_linesearch()) {
      t *= shrink_rate_;
      theta = select_theta(t);
      y = (1 - theta) * x_old + theta * v;
      x = y - t * problem.EvalGrad(y);
      x = problem.EvalProximator(x, t);
    }
    // SECT: Update v, theta, step_length
    theta_old = theta;
    step_length_old = t;
    v = 1 / theta * (x - x_old) + x_old;
    tk_.push_back(t);
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
