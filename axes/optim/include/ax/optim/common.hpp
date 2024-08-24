#pragma once

#include "ax/math/common.hpp"
#include "ax/math/linalg.hpp"
#include "ax/math/sparse.hpp"
#include "ax/utils/common.hpp"

namespace ax::optim {

using Variable = math::RealMatrixX;            // type of variable in optimization.
using Gradient = Variable;                     // type of gradient is the same as variable.
using DenseHessian = math::RealMatrixX;        // type of dense hessian.
using SparseHessian = math::RealSparseMatrix;  // type of sparse hessian.

/****************************** Function Handles ******************************/
using ConvergeVarFn = std::function<Real(const Variable&, const Variable&)>;
using ConvergeGradFn = std::function<Real(const Gradient&, const Gradient&)>;
using VerboseFn = std::function<void(Index, const Variable&, Real)>;
using EnergyFn = std::function<Real(const Variable&)>;
using GradFn = std::function<Gradient(const Variable&)>;
using HessianFn = std::function<DenseHessian(const Variable&)>;
using SparseHessianFn = std::function<SparseHessian(const Variable&)>;
using ProximatorFn = std::function<math::RealMatrixX(const Variable&, Real)>;

template <typename NormType = math::l2_t>
Real default_converge_grad(const Gradient&, const Gradient& grad) {
  return math::norm(grad, NormType{});
}

template <typename NormType = math::l2_t>
Real default_converge_var(const Variable& x0, const Variable& x1) {
  return math::norm(x1 - x0, NormType{});
}

class OptProblem {
public:
  OptProblem();
  AX_DECLARE_CONSTRUCTOR(OptProblem, default, default);

  /****************************** Evaluation ******************************/
  Real EvalEnergy(const Variable& x) const;
  Gradient EvalGrad(const Variable& x) const;
  DenseHessian EvalHessian(const Variable& x) const;
  SparseHessian EvalSparseHessian(const Variable& x) const;
  Real EvalConvergeVar(const Variable& x0, const Variable& x1) const;
  Real EvalConvergeGrad(const Gradient& x, const Gradient& grad) const;
  void EvalVerbose(Index iter, const Variable& x, Real energy) const;
  Variable EvalProximator(const Variable& x, Real step_length) const;

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

/************************* Optimization Result **************************/
struct OptResult {
  // Optimal x
  Variable x_opt_;

  // Optimal energy
  Real f_opt_{math::nan<Real>};
  Real step_length_{1.0};  // for linesarchers.
  // For Iterative Solver:
  Index n_iter_{0};
  // Indicates whether the optimization algorithm has converged.
  bool converged_grad_{false};  // Stronge convergence criteria.
  bool converged_var_{false};   // Weak convergence criteria.
  bool converged_{false};       // Any criteria, reserved for Linesearch.
  std::string err_msg_;         ///< Error message, reason for converge failure.

  OptResult() = default;

  OptResult(Variable const& x_opt, Real f_opt, Index n_iter)
      : x_opt_(x_opt), f_opt_(f_opt), n_iter_(n_iter) {}

  std::pair<Variable, Real> GetResult() const;

  // Convergence check
  bool IsConverged() const;
  explicit operator bool() const;

  static OptResult NotConverged(std::string const& msg = "");

  static OptResult NotConverged(Variable x_final, Real f_final, Index n_iter,
                                std::string const& msg);

  static OptResult Converged(Variable x_opt, Real f_opt, Index n_iter, bool converged_grad,
                             bool converged_var);

  static OptResult ConvergedLinesearch(Variable x_opt, Real f_opt, Index n_iter, Real step_length);
};

std::ostream& operator<<(std::ostream& os, OptResult const& result);

}  // namespace ax::optim
