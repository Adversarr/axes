#pragma once

#include "axes/math/common.hpp"
#include "axes/math/linalg.hpp"
#include "axes/math/sparse.hpp"
#include "axes/utils/common.hpp"

namespace ax::optim {

/****************************** Function Handles ******************************/
using ConvergeVarFn
    = std::function<real(const math::vecxr&, const math::vecxr&)>;
using ConvergeGradFn
    = std::function<real(const math::vecxr&, const math::vecxr&)>;
using VerboseFn = std::function<void(idx, const math::vecxr&, const real)>;
using EnergyFn = std::function<real(const math::vecxr&)>;
using GradFn = std::function<math::vecxr(const math::vecxr&)>;
using HessianFn = std::function<math::matxxr(const math::vecxr&)>;
using SparseHessianFn = std::function<math::sp_matxxr(math::vecxr const&)>;
template <typename NormType = math::l2_t>
inline real default_converge_grad(math::vecxr const&, math::vecxr const& grad) {
  return math::norm(grad, NormType{});
}

template <typename NormType = math::l2_t>
inline real default_converge_var(math::vecxr const& x0, math::vecxr const& x1) {
  return math::norm(x1 - x0, NormType{});
}

class OptProblem {
public:
  OptProblem();
  AX_DECLARE_CONSTRUCTOR(OptProblem, default, default);

  /****************************** Evaluation ******************************/
  real EvalEnergy(math::vecxr const& x) const;
  math::vecxr EvalGrad(math::vecxr const& x) const;
  math::matxxr EvalHessian(math::vecxr const& x) const;
  math::sp_matxxr EvalSparseHessian(math::vecxr const& x) const;
  real EvalConvergeVar(math::vecxr const& x0, math::vecxr const& x1) const;
  real EvalConvergeGrad(math::vecxr const& x, math::vecxr const& grad) const;
  void EvalVerbose(idx iter, math::vecxr const& x, real f) const;

  /****************************** Setters ******************************/
  OptProblem& SetEnergy(EnergyFn const& energy);

  OptProblem& SetGrad(GradFn const& grad);

  OptProblem& SetHessian(HessianFn const& hessian);

  OptProblem& SetSparseHessian(SparseHessianFn const& sparse_hessian);

  OptProblem& SetConvergeVar(ConvergeVarFn const& converge_var);

  OptProblem& SetConvergeGrad(ConvergeGradFn const& converge_grad);

  OptProblem& SetVerbose(VerboseFn const& verbose);

  /****************************** Check Valid ******************************/
  bool HasEnergy() const;

  bool HasGrad() const;

  bool HasHessian() const;

  bool HasSparseHessian() const;

  bool HasConvergeVar() const;

  bool HasConvergeGrad() const;

  bool HasVerbose() const;

private:
  EnergyFn energy_{nullptr};
  GradFn grad_{nullptr};
  HessianFn hessian_{nullptr};
  SparseHessianFn sparse_hessian_{nullptr};
  ConvergeVarFn converge_var_{nullptr};
  ConvergeGradFn converge_grad_{nullptr};
  VerboseFn verbose_{nullptr};
};

/****************************** Optimization Result
 * ******************************/
struct OptResultImpl {
  // Optimal x
  math::vecxr x_opt_;
  // Optimal energy
  real f_opt_;
  // Indicates whether the optimization algorithm has converged.
  bool converged_grad_;
  bool converged_var_;
  bool converged_;

  // For Iterative Solver:
  idx n_iter_;

  OptResultImpl() = default;

  OptResultImpl(math::vecxr const& x_opt, real f_opt, idx n_iter)
      : x_opt_(x_opt), f_opt_(f_opt), n_iter_(n_iter) {}

  std::pair<math::vecxr, real> GetResult() const;
};

using OptResult = StatusOr<OptResultImpl>;

std::ostream& operator<<(std::ostream& os, OptResultImpl const& result);

}  // namespace ax::optim
