#include "ax/optim2/problem.hpp"

#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/create_buffer.hpp"
#include "ax/math/buffer_blas.hpp"

namespace ax::optim2 {

ProblemBase::ProblemBase(RealBufferView variables, ConstRealBufferView gradient)
    : variables_(variables), gradient_(gradient) {}

ProblemBase::ProblemBase(RealBufferView variables, ConstRealBufferView gradient,
                         math::ConstRealSparseMatrixPtr hessian)
    : variables_(variables), gradient_(gradient), hessian_(std::move(hessian)) {}

void ProblemBase::StepForwardActual() {
  // make sure the energy/gradient is up to date.
  UpdateGradient();
  UpdateEnergy();

  // test the convergence criteria.
  // to get the variable change, we reuse backup buffer.
  auto [back_var, back_grad] = make_view(backup_var_, backup_grad_);

  // back_var = -variables + back_var
  math::buffer_blas::axpy(-1.0, variables_, back_var);
  // norm = |back_var|
  last_variable_change_ = math::buffer_blas::norm(back_var);
  grad_norm_ = math::buffer_blas::norm(gradient_);
  is_grad_norm_up_to_date_ = true;

  // copy the current value to backup.
  copy(backup_var_->ConstView(), variables_);
  backup_energy_ = energy_;
  backup_grad_norm_ = grad_norm_;
  copy(backup_grad_->ConstView(), gradient_);
}

void ProblemBase::StepTemporary(ConstRealBufferView new_value, Real alpha, Real beta) {
  AX_EXPECTS(new_value.Shape() == variables_.Shape());

  math::buffer_blas::scal(beta, variables_);
  math::buffer_blas::axpy(alpha, new_value, variables_);
  MarkVariableChanged();
}

void ProblemBase::BeginOptimize() {
  backup_var_ = ensure_buffer<Real>(backup_var_, variables_.Device(), variables_.Shape());
  backup_grad_ = ensure_buffer<Real>(backup_grad_, gradient_.Device(), gradient_.Shape());
}

void ProblemBase::EndOptimize() {
  // do nothing.
}

void ProblemBase::StepBack() {
  copy(variables_, backup_var_->ConstView());
}

void ProblemBase::MarkVariableChanged() {
  // Do nothing.
}

void ProblemBase::UpdateHessian() {
  if (hessian_ != nullptr) {
    AX_WARN("Hessian is available but UpdateHessian is not implemented.");
  }
}

Real ProblemBase::GetEnergy() const {
  return energy_;
}

ConstRealBufferView ProblemBase::GetVariables() const {
  return variables_;
}

ConstRealBufferView ProblemBase::GetGradient() const {
  return gradient_;
}

const_shared_not_null<math::RealCompressedMatrixBase> ProblemBase::GetHessian() const {
  AX_EXPECTS(hessian_ != nullptr);
  return hessian_;
}

void ProblemBase::OnStep(bool is_in_linesearch, size_t iteration) noexcept {
  if (is_in_linesearch) {
    AX_INFO("ls: {:2}, energy={:12.6e}", iteration, energy_);
  } else {
    AX_INFO("iter: {:2}, energy={:12.6e}", iteration, energy_);
  }
}

Real ProblemBase::GetGaridentNorm() {
  if (!is_grad_norm_up_to_date_) {
    grad_norm_ = math::buffer_blas::norm(gradient_);
    is_grad_norm_up_to_date_ = true;
  }
  return grad_norm_;
}

Real ProblemBase::GetLastVariableChange() const {
  return last_variable_change_;
}

Real ProblemBase::GetBackupEnergy() const {
  return backup_energy_;
}

Real ProblemBase::GetBackupGradientNorm() const {
  return backup_grad_norm_;
}

ConstRealBufferView ProblemBase::GetBackupVariables() const {
  return backup_var_->ConstView();
}

ConstRealBufferView ProblemBase::GetBackupGradient() const {
  return backup_grad_->ConstView();
}
}  // namespace ax::optim2