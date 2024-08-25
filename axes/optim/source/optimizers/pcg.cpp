#include "ax/optim/optimizers/pcg.hpp"

#include "ax/core/logging.hpp"

namespace ax::optim {

Optimizer_NonlinearCg::Optimizer_NonlinearCg() {
  linesearch_ = LinesearchBase::Create(LineSearchKind::kBacktracking);
}

Optimizer_NonlinearCg::~Optimizer_NonlinearCg() = default;

OptimizerKind Optimizer_NonlinearCg::GetKind() const {
  return OptimizerKind::kNonlinearCg;
}

utils::Options Optimizer_NonlinearCg::GetOptions() const {
  utils::Options options = OptimizerBase::GetOptions();
  options["strategy"] = utils::reflect_name(strategy_).value_or("???");
  if (linesearch_) {
    options["linesearch"] = utils::reflect_name(linesearch_->GetKind()).value_or("???");
    options["linesearch_opt"] = linesearch_->GetOptions();
  }
  options["restart_period"] = restart_period_;
  return options;
}

void Optimizer_NonlinearCg::SetStrategy(NonlinearCgStrategy strategy) {
  strategy_ = strategy;
}

void Optimizer_NonlinearCg::SetOptions(utils::Options const& option) {
  OptimizerBase::SetOptions(option);
  AX_SYNC_OPT(option, Index, restart_period);
  utils::extract_and_create<LinesearchBase, LineSearchKind>(option, "linesearch", linesearch_);
  utils::extract_tunable(option, "linesearch_opt", linesearch_.get());
  AX_SYNC_OPT_ENUM(option, NonlinearCgStrategy, strategy_, strategy);
}

OptResult Optimizer_NonlinearCg::Optimize(OptProblem const& prob, const Variable& x0) const {
  // check the problem: grad, energy
  AX_THROW_IF_FALSE(prob.HasEnergy(), "Problem does not have energy.");
  AX_THROW_IF_FALSE(prob.HasGrad(), "Problem does not have gradient.");
  if (!precond_) {
    precond_ = details::Dummy();
  }

  // initialize the variables
  Variable x = x0, x_old = x0;

  Gradient grad = prob.EvalGrad(x);  // grad = \nabla f(x)
  Gradient s = precond_(x, grad);    // s = M^{-1} * grad
  Variable search_dir = -s;          // initial search direction
  Real f = prob.EvalEnergy(x);

  Index iter = 0;
  Real delta_new = math::dot(grad, s);
  Index cnt_restart = 0;
  bool converged_grad = false, converged_var = false;
  Real expect_descent = 0;
  for (; iter < max_iter_; ++iter) {
    if (verbose_) {
      prob.EvalVerbose(iter, x, f);
      AX_INFO("iter {:3}: f={:12.6e}, |grad|={:12.6e}, |s|={:12.6e}, delta={:12.6e}", iter, f,
              math::norm(grad), math::norm(s), delta_new);
    }

    converged_grad = prob.EvalConvergeGrad(x, grad) < tol_grad_;
    converged_var = iter > 0 && prob.EvalConvergeVar(x_old, x) < tol_var_;
    if (converged_grad || converged_var) {
      break;
    }

    // Launch Linesearch:
    auto result = linesearch_->Optimize(prob, x, grad, search_dir);
    if (!result) {
      // Linesearch not converge.
      AX_ERROR("Linesearch failed: {}", result.err_msg_);
      break;
    }

    // Update x, grad.
    x_old = std::move(x);                  //    x_old = x[n - 1]
    x = std::move(result.x_opt_);          //        x = x[n]
    Gradient grad_old = prob.EvalGrad(x);  // grad_old = grad[n - 1]
    grad.swap(grad_old);                   //     grad = grad[n]
    f = result.f_opt_;

    // Update search_dir, s. s is the preconditioned gradient.
    Real delta_old = delta_new;           // grad[n-1] dot s[n-1]
    Real delta_mid = math::dot(grad, s);  //   grad[n] dot s[n-1]
    s = precond_(x, grad);                //         s  =  s[n]
    delta_new = math::dot(grad, s);       //   grad[n] dot s[n]
    Real beta = 0;
    switch (strategy_) {
      case NonlinearCgStrategy::kFletcherReeves:
        beta = delta_new / delta_old;
        break;
      case NonlinearCgStrategy::kPolakRibiere:
        beta = (delta_new - delta_mid) / delta_old;
        break;
      case NonlinearCgStrategy::kPolakRibiereClamped:
        beta = std::max(0.0, (delta_new - delta_mid) / delta_old);
        break;

      case NonlinearCgStrategy::kHestenesStiefel: {
        // currently, expected_descent=-grad[n-1] dot search_dir[n-1]
        Real ed_mid = math::dot(grad, search_dir);  // ed_mid = grad[n] dot search_dir[n-1]
        beta = (delta_new - delta_mid) / (ed_mid - expect_descent);
        break;
      }

      case NonlinearCgStrategy::kHestenesStiefelClamped: {
        Real ed_mid = math::dot(grad, search_dir);  // ed_mid = grad[n] dot search_dir[n-1]
        beta = std::max(0.0, (delta_new - delta_mid) / (ed_mid - expect_descent));
        break;
      }

      case NonlinearCgStrategy::kDaiYuan: {
        Real ed_mid = math::dot(grad, search_dir);  // ed_mid = grad[n] dot search_dir[n-1]
        beta = delta_new / (ed_mid - expect_descent);
        break;
      }

      default:
        AX_UNREACHABLE();
    }

    search_dir = -s + beta * search_dir;
    expect_descent = math::dot(search_dir, grad);  // expect_descent = -grad[n] dot search_dir[n]
    fmt::print("beta={:12.6e}, stepsize={:12.6e}\n", beta, result.step_length_);

    // Restart.
    ++cnt_restart;
    bool need_restart = cnt_restart >= restart_period_;
    if (expect_descent >= 0) {
      AX_WARN("In iteration {:3}, Dot[search_dir, grad] >= 0, force restart!", iter);
      need_restart = true;
    }

    if (need_restart) {
      cnt_restart = 0;
      search_dir = -s;
    }
  }

  if (iter >= max_iter_) {
    return OptResult::NotConverged("Max iteration reached.");
  } else {
    AX_CHECK(converged_grad || converged_var, "Invalid convergence status.");
    return OptResult::Converged(x, prob.EvalEnergy(x), iter, converged_grad, converged_var);
  }
}

namespace details {
NonlinearCgPreconditioner Dummy() noexcept {
  return [](const Variable& /*x*/, const Gradient& g) {
    return g;
  };
}
}  // namespace details
}  // namespace ax::optim
