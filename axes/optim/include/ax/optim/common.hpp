#pragma once

#include "ax/math/common.hpp"
#include "ax/math/linalg.hpp"
#include "ax/math/sparse.hpp"
#include "ax/utils/common.hpp"

namespace ax::optim {

using Variable = math::matxxr;       // type of variable in optimization.
using Gradient = Variable;           // type of gradient is the same as variable.
using DenseHessian = math::matxxr;   // type of dense hessian.
using SparseHessian = math::spmatr;  // type of sparse hessian.

/****************************** Function Handles ******************************/
using ConvergeVarFn = std::function<real(const Variable&, const Variable&)>;
using ConvergeGradFn = std::function<real(const Gradient&, const Gradient&)>;
using VerboseFn = std::function<void(Index, const Variable&, real)>;
using EnergyFn = std::function<real(const Variable&)>;
using GradFn = std::function<Gradient(const Variable&)>;
using HessianFn = std::function<DenseHessian(const Variable&)>;
using SparseHessianFn = std::function<SparseHessian(const Variable&)>;
using ProximatorFn = std::function<math::matxxr(const Variable&, real)>;

template <typename NormType = math::l2_t>
real default_converge_grad(const Gradient &, const Gradient& grad) {
  return math::norm(grad, NormType{});
}

template <typename NormType = math::l2_t>
real default_converge_var(const Variable& x0, const Variable& x1) {
  return math::norm(x1 - x0, NormType{});
}

class OptProblem {
public:
  OptProblem();
  AX_DECLARE_CONSTRUCTOR(OptProblem, default, default);

  /****************************** Evaluation ******************************/
  real EvalEnergy(const Variable& x) const;
  Gradient EvalGrad(const Variable& x) const;
  DenseHessian EvalHessian(const Variable& x) const;
  SparseHessian EvalSparseHessian(const Variable& x) const;
  real EvalConvergeVar(const Variable& x0, const Variable& x1) const;
  real EvalConvergeGrad(const Gradient& grad0, const Gradient& grad1) const;
  void EvalVerbose(Index iter, const Variable& x, real energy) const;
  Variable EvalProximator(const Variable& x, real step_length) const;

  /****************************** Setters ******************************/
  OptProblem& SetEnergy(EnergyFn const& energy);
  OptProblem& SetGrad(GradFn const& grad);
  OptProblem& SetHessian(HessianFn const& hessian);
  OptProblem& SetSparseHessian(SparseHessianFn const& sparse_hessian);
  OptProblem& SetConvergeVar(ConvergeVarFn const& converge_var);
  OptProblem& SetConvergeGrad(ConvergeGradFn const& converge_grad);
  OptProblem& SetVerbose(VerboseFn const& verbose);
  OptProblem& SetProximator(ProximatorFn const& proximator);

  /****************************** Check Valid ******************************/
  bool HasEnergy() const;

  bool HasGrad() const;

  bool HasHessian() const;

  bool HasSparseHessian() const;

  bool HasConvergeVar() const;

  bool HasConvergeGrad() const;

  bool HasVerbose() const;

  bool HasProximator() const;

private:
  EnergyFn energy_{nullptr};
  GradFn grad_{nullptr};
  HessianFn hessian_{nullptr};
  SparseHessianFn sparse_hessian_{nullptr};
  ConvergeVarFn converge_var_{nullptr};
  ConvergeGradFn converge_grad_{nullptr};
  VerboseFn verbose_{nullptr};
  ProximatorFn proximator_{nullptr};
};

/****************************** Optimization Result
 * ******************************/
struct OptResult {
  // Optimal x
  Variable x_opt_;

  // Optimal energy
  real f_opt_;
  real step_length_{1.0}; // for linesarchers.
  // For Iterative Solver:
  Index n_iter_;
  // Indicates whether the optimization algorithm has converged.
  bool converged_grad_{false};
  bool converged_var_{false};
  bool converged_{false};

  std::string err_msg_; ///< Error message, reason for converge failure.

  OptResult() = default;

  OptResult(Variable const& x_opt, real f_opt, Index n_iter)
      : x_opt_(x_opt), f_opt_(f_opt), n_iter_(n_iter) {}

  std::pair<Variable, real> GetResult() const;
};

std::ostream& operator<<(std::ostream& os, OptResult const& result);

}  // namespace ax::optim
