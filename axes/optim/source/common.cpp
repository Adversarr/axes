#include "ax/optim/common.hpp"

#include "ax/core/logging.hpp"
#include "ax/core/excepts.hpp"

namespace ax::optim {

OptProblem::OptProblem()
    : converge_var_(default_converge_var<math::l2_t>),
      converge_grad_(default_converge_grad<math::l2_t>) {}

std::pair<Variable, real> OptResult::GetResult() const {
  return {x_opt_, f_opt_};
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

OptProblem& OptProblem::SetSparseHessian(
    SparseHessianFn const& sparse_hessian) {
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

real OptProblem::EvalEnergy(const Variable& x) const {
  AX_THROW_IF_NULL(energy_, "Energy Fn is not set.");
  return energy_(x);
}

void OptProblem::EvalVerbose(idx iter, const Variable& x, real f) const {
  if (verbose_) {
    verbose_(iter, x, f);
  }
}

real OptProblem::EvalConvergeVar(const Variable& x0,
                                 const Variable& x1) const {
  AX_THROW_IF_NULL(converge_var_, "Converge Var Fn is not set.");
  return converge_var_(x0, x1);
}

real OptProblem::EvalConvergeGrad(const Variable& x,
                                  const Variable& grad) const {
  AX_THROW_IF_NULL(converge_grad_, "Converge Grad Fn is not set.");
  return converge_grad_(x, grad);
}

Variable OptProblem::EvalProximator(const Variable& x,
                                       real step_length) const {
  AX_THROW_IF_NULL(proximator_, "Proximator Fn is not set.");
  if (proximator_) {
    return proximator_(x, step_length);
  }
  return x;
}

bool OptProblem::HasEnergy() const { return static_cast<bool>(energy_); }

bool OptProblem::HasGrad() const { return static_cast<bool>(grad_); }

bool OptProblem::HasHessian() const { return static_cast<bool>(hessian_); }

bool OptProblem::HasSparseHessian() const {
  return static_cast<bool>(sparse_hessian_);
}

bool OptProblem::HasConvergeVar() const {
  return static_cast<bool>(converge_var_);
}

bool OptProblem::HasConvergeGrad() const {
  return static_cast<bool>(converge_grad_);
}

bool OptProblem::HasVerbose() const { return static_cast<bool>(verbose_); }

bool OptProblem::HasProximator() const {
  return static_cast<bool>(proximator_);
}

}  // namespace ax::optim
