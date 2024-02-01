#include "axes/optim/common.hpp"

#include "axes/core/echo.hpp"

namespace ax::optim {

std::pair<math::vecxr, real> OptResultImpl::GetResult() const { return {x_opt_, f_opt_}; }

std::ostream& operator<<(std::ostream& os, OptResultImpl const& result) {
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

bool OptProblem::CheckConverge(math::vecxr const& x0, math::vecxr const& x1,
                               math::vecxr const& grad) const {
  // TODO: Check before optimize
  CHECK(converge_var_ || converge_grad_) << "No convergence criteria is set.";
  return CheckConvergeVar(x0, x1) || CheckConvergeGrad(x1, grad);
}
bool OptProblem::CheckConvergeGrad(math::vecxr const& x, math::vecxr const& grad) const {
  if (!converge_grad_) {
    return false;
  }
  return converge_grad_(x, grad);
}

bool OptProblem::CheckConvergeVar(math::vecxr const& x0, math::vecxr const& x1) const {
  if (!converge_var_) {
    return false;
  }
  return converge_var_(x0, x1);
}

math::sp_matxxr OptProblem::EvalSparseHessian(math::vecxr const& x) const {
  CHECK(sparse_hessian_) << "Sparse Hessian Fn is not set.";
  return sparse_hessian_(x);
}

math::matxxr OptProblem::EvalHessian(math::vecxr const& x) const {
  CHECK(hessian_) << "Hessian Fn is not set.";
  return hessian_(x);
}

math::vecxr OptProblem::EvalGrad(math::vecxr const& x) const {
  CHECK(grad_) << "Grad Fn is not set.";
  return grad_(x);
}

real OptProblem::EvalEnergy(math::vecxr const& x) const {
  // TODO: Check Before Optimize
  CHECK(energy_) << "Energy Fn is not set.";
  return energy_(x);
}

}  // namespace ax::optim
