#pragma once
#include "ax/core/buffer/buffer_view.hpp"
#include "ax/core/gsl.hpp"
#include "ax/math/sparse_matrix/linsys/common.hpp"
#include "common.hpp"

namespace ax::optim2 {

class ProblemBase {
public:
  ProblemBase(RealBufferView variables, ConstRealBufferView gradient);
  ProblemBase(RealBufferView variables, ConstRealBufferView gradient,
              math::ConstRealSparseMatrixPtr hessian);
  virtual ~ProblemBase();

  // Flush the variables to backup, and test the convergence criteria.
  void StepForwardActual();

  // Only do the step, no backup, useful for linesearch and acceleration methods. It will blend
  // between current value and the new value.
  // computes: var <- alpha * new_value + beta * var
  void StepTemporary(ConstRealBufferView new_value, Real alpha, Real beta);

  // initialize the backup buffer.
  void BeginOptimize();
  // finalize, copy the backup to the variables if the fn value is lower
  void EndOptimize();

  // Restore the variables from backup, useful for linesearch to step back.
  // computes: var <- backup
  void StepBack();

  // the default version is no-op.
  virtual void MarkVariableChanged();
  // lazy evaluation of energy
  virtual void UpdateEnergy() = 0;
  // lazy evaluation of gradient
  virtual void UpdateGradient() = 0;
  // lazy evaluation of hessian, default to assume the hessian does not change.
  virtual void UpdateHessian();

  // callback function for each OPTIMIZER step.
  virtual void OnStep(bool is_in_linesearch, size_t iteration) noexcept;

  // returns the variables.
  ConstRealBufferView GetVariables() const;
  // returns the energy.
  Real GetEnergy() const;
  // returns the gradient norm.
  Real GetGaridentNorm();
  // returns the gradient.
  ConstRealBufferView GetGradient() const;
  // returns the hessian.
  const_shared_not_null<math::RealCompressedMatrixBase> GetHessian() const;
  Real GetLastVariableChange() const;

  Real GetBackupEnergy() const;
  Real GetBackupGradientNorm() const;
  ConstRealBufferView GetBackupVariables() const;
  ConstRealBufferView GetBackupGradient() const;

protected:
  // Workspace.
  RealBufferView variables_;  // can used by linesearch.
  Real energy_{0};
  Real grad_norm_{0};
  bool is_grad_norm_up_to_date_{false};
  ConstRealBufferView gradient_;
  math::ConstRealSparseMatrixPtr hessian_;
  Real last_variable_change_{0};

  // Linesearchers should not change the following part.
  BufferPtr<Real> backup_var_;
  Real backup_energy_{math::nan<>};     // must be up to date.
  Real backup_grad_norm_{math::nan<>};  // must be up to date.
  BufferPtr<Real> backup_grad_;
  // we do not need to backup the hessian.

  // For a typical problem, the hessian does change its sparse pattern.
  // therefore, we just need to call the AnalysePattern() once.
  bool hessian_change_topo_{false};
};

}  // namespace ax::optim2