#include "ax/optim/common.hpp"

#include "ax/core/excepts.hpp"
#include "ax/core/logging.hpp"

namespace ax::optim {

OptProblem::OptProblem()
    : converge_var_(default_converge_var<math::l2_t>),
      converge_grad_(default_converge_grad<math::l2_t>) {}

std::pair<Variable, Real> OptResult::GetResult() const {
  return {x_opt_, f_opt_};
}

bool OptResult::IsConverged() const {
  return converged_;
}

OptResult::operator bool() const {
  return IsConverged();
}

OptResult OptResult::NotConverged(std::string const& msg) {
  OptResult result;
  result.err_msg_ = msg;
  result.converged_ = result.converged_grad_ = result.converged_var_ = false;
  return result;
}

OptResult OptResult::NotConverged(Variable x_final, Real f_final, Index n_iter,
                                  std::string const& msg) {
  OptResult result;
  result.x_opt_ = std::move(x_final);
  result.f_opt_ = f_final;
  result.n_iter_ = n_iter;
  result.err_msg_ = msg;
  return result;
}

OptResult OptResult::Converged(Variable x_opt, Real f_opt, Index n_iter, bool converged_grad,
                               bool converged_var) {
  OptResult result;
  result.x_opt_ = std::move(x_opt);
  result.f_opt_ = f_opt;
  result.n_iter_ = n_iter;
  result.converged_grad_ = converged_grad;
  result.converged_var_ = converged_var;
  result.converged_ = converged_grad || converged_var;
  return result;
}

OptResult OptResult::ConvergedLinesearch(Variable x_opt, Real f_opt, Index n_iter,
                                         Real step_length) {
  OptResult result;
  result.x_opt_ = std::move(x_opt);
  result.f_opt_ = f_opt;
  result.step_length_ = step_length;
  result.n_iter_ = n_iter;
  result.converged_grad_ = result.converged_var_ = false;
  result.converged_ = true;
  return result;
}

std::ostream& operator<<(std::ostream& os, OptResult const& result) {
  os << "Optimization Result:\n";
  os << "  x_opt: " << result.x_opt_.transpose() << "\n";
  os << "  f_opt: " << result.f_opt_ << "\n";
  os << "  n_iter: " << result.n_iter_ << "\n";
  return os;
}

OptProblem& OptProblem::SetEnergy(EnergyFn const& energy) {
  energy_ = energy;
  return *this;
}

OptProblem& OptProblem::SetGrad(GradFn const& grad) {
  grad_ = grad;
  return *this;
}

OptProblem& OptProblem::SetHessian(HessianFn const& hessian) {
  hessian_ = hessian;
  return *this;
}

OptProblem& OptProblem::SetSparseHessian(SparseHessianFn const& sparse_hessian) {
  sparse_hessian_ = sparse_hessian;
  return *this;
}

OptProblem& OptProblem::SetConvergeVar(ConvergeVarFn const& converge_var) {
  converge_var_ = converge_var;
  return *this;
}

OptProblem& OptProblem::SetConvergeGrad(ConvergeGradFn const& converge_grad) {
  converge_grad_ = converge_grad;
  return *this;
}

OptProblem& OptProblem::SetVerbose(VerboseFn const& verbose) {
  verbose_ = verbose;
  return *this;
}

OptProblem& OptProblem::SetProximator(ProximatorFn const& proximator) {
  proximator_ = proximator;
  return *this;
}

SparseHessian OptProblem::EvalSparseHessian(const Variable& x) const {
  AX_THROW_IF_NULL(sparse_hessian_, "Sparse Hessian Fn is not set.");
  return sparse_hessian_(x);
}

DenseHessian OptProblem::EvalHessian(const Variable& x) const {
  AX_THROW_IF_NULL(hessian_, "Hessian Fn is not set.");
  return hessian_(x);
}

Gradient OptProblem::EvalGrad(const Variable& x) const {
  AX_THROW_IF_NULL(grad_, "Grad Fn is not set.");
  return grad_(x);
}

Real OptProblem::EvalEnergy(const Variable& x) const {
  AX_THROW_IF_NULL(energy_, "Energy Fn is not set.");
  return energy_(x);
}

void OptProblem::EvalVerbose(Index iter, const Variable& x, Real f) const {
  if (verbose_) {
    verbose_(iter, x, f);
  }
}

Real OptProblem::EvalConvergeVar(const Variable& x0, const Variable& x1) const {
  AX_THROW_IF_NULL(converge_var_, "Converge Var Fn is not set.");
  return converge_var_(x0, x1);
}

Real OptProblem::EvalConvergeGrad(const Variable& x, const Variable& grad) const {
  AX_THROW_IF_NULL(converge_grad_, "Converge Grad Fn is not set.");
  return converge_grad_(x, grad);
}

Variable OptProblem::EvalProximator(const Variable& x, Real step_length) const {
  AX_THROW_IF_NULL(proximator_, "Proximator Fn is not set.");
  if (proximator_) {
    return proximator_(x, step_length);
  }
  return x;
}

bool OptProblem::HasEnergy() const {
  return static_cast<bool>(energy_);
}

bool OptProblem::HasGrad() const {
  return static_cast<bool>(grad_);
}

bool OptProblem::HasHessian() const {
  return static_cast<bool>(hessian_);
}

bool OptProblem::HasSparseHessian() const {
  return static_cast<bool>(sparse_hessian_);
}

bool OptProblem::HasConvergeVar() const {
  return static_cast<bool>(converge_var_);
}

bool OptProblem::HasConvergeGrad() const {
  return static_cast<bool>(converge_grad_);
}

bool OptProblem::HasVerbose() const {
  return static_cast<bool>(verbose_);
}

bool OptProblem::HasProximator() const {
  return static_cast<bool>(proximator_);
}

}  // namespace ax::optim
