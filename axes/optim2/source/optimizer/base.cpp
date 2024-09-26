#include "ax/optim2/optimizer/base.hpp"

#include "ax/optim2/linesearch/base.hpp"
#include "ax/optim2/optimizer/gradient_descent.hpp"
#include "ax/optim2/optimizer/lbfgs.hpp"
#include "ax/optim2/optimizer/ncg.hpp"
#include "ax/optim2/optimizer/newton.hpp"

namespace ax::optim2 {

OptimizerBase::OptimizerBase() = default;

OptimizerBase::~OptimizerBase() = default;

void OptimizerBase::SetLinesearch(LineSearchPtr ls) {
  linesearch_ = std::move(ls);
}

std::unique_ptr<OptimizerBase> OptimizerBase::Create(OptimizerKind kind) {
  switch (kind) {
    case OptimizerKind::Newton:
      return std::make_unique<Optimizer_Newton>();

    case OptimizerKind::Lbfgs:
      return std::make_unique<Optimizer_LBFGS>();

    case OptimizerKind::GradientDescent:
      return std::make_unique<Optimizer_GradientDescent>();

    case OptimizerKind::NonlinearCg:
      return std::make_unique<Optimizer_NonlinearCg>();

    case OptimizerKind::Fista:
      AX_NOT_IMPLEMENTED();

    default:
      AX_THROW_INVALID_ARGUMENT("Unknown optimizer kind.");
  }
  AX_UNREACHABLE();
}

utils::Options OptimizerBase::GetOptions() const {
  auto opt = Tunable::GetOptions();
  opt["max_iter"] = max_iter_;
  opt["tol_grad"] = tol_grad_;
  opt["tol_var"] = tol_var_;
  opt["verbose"] = verbose_;
  return opt;
}

void OptimizerBase::SetOptions(utils::Options const& option) {
  Tunable::SetOptions(option);
  AX_SYNC_OPT(option, size_t, max_iter);
  AX_SYNC_OPT(option, Real, tol_grad);
  AX_SYNC_OPT(option, Real, tol_var);
  AX_SYNC_OPT(option, bool, verbose);
}

bool OptimizerBase::TestCriteria(const OptimizeParam& param, OptimizeResult& result) {
  bool any_converged = false;
  if (problem_->GetGaridentNorm() < param.tol_grad_.value_or(tol_grad_)) {
    result.converged_grad_ = true;
    any_converged = true;
  }
  if (result.n_iter_ > 0 && problem_->GetLastVariableChange() < param.tol_var_.value_or(tol_var_)) {
    result.converged_var_ = true;
    any_converged = true;
  }
  return any_converged;
}

void OptimizerBase::CheckInputParam(const OptimizeParam& param) const {
  if (auto max_iter = param.max_iter_.value_or(max_iter_); max_iter <= 0) {
    AX_THROW_INVALID_ARGUMENT("max_iter must be positive. got {}", max_iter);
  }

  if (auto tol_grad = param.tol_grad_.value_or(tol_grad_); tol_grad <= 0) {
    AX_THROW_INVALID_ARGUMENT("tol_grad must be positive. got {:12.6e}", tol_grad);
  }

  if (auto tol_var = param.tol_var_.value_or(tol_var_); tol_var <= 0) {
    AX_THROW_INVALID_ARGUMENT("tol_var must be positive. got {:12.6e}", tol_var);
  }
}

}  // namespace ax::optim2
