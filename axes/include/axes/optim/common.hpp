#pragma once

#include "axes/math/common.hpp"
#include "axes/math/sparse.hpp"

namespace ax::optim {

/****************************** Function Handles ******************************/
using ConvergeVarFn = std::function<bool(const math::vecxr&, const math::vecxr&)>;
using ConvergeGradFn = std::function<bool(const math::vecxr&, const math::vecxr&)>;
using VerboseFn = std::function<void(idx, const math::vecxr&, const real)>;
using EnergyFn = std::function<real(const math::vecxr&)>;
using GradFn = std::function<math::vecxr(const math::vecxr&)>;
using HessianFn = std::function<math::matxxr(const math::vecxr&)>;
using SparseHessianFn = std::function<math::sp_matxxr(math::vecxr const&)>;

class OptProblem {
public:
  OptProblem() = default;
  OptProblem(OptProblem const&) = default;

  /****************************** Evaluation ******************************/
  real EvalEnergy(math::vecxr const& x) const;

  math::vecxr EvalGrad(math::vecxr const& x) const;

  math::matxxr EvalHessian(math::vecxr const& x) const;

  math::sp_matxxr EvalSparseHessian(math::vecxr const& x) const;

  bool CheckConvergeVar(math::vecxr const& x0, math::vecxr const& x1) const;

  bool CheckConvergeGrad(math::vecxr const& x, math::vecxr const& grad) const;

  bool CheckConverge(math::vecxr const& x0, math::vecxr const& x1, math::vecxr const& grad) const;

  /****************************** Setters ******************************/
  OptProblem& SetEnergy(EnergyFn const& energy);

  OptProblem& SetGrad(GradFn const& grad);

  OptProblem& SetHessian(HessianFn const& hessian);

  OptProblem& SetSparseHessian(SparseHessianFn const& sparse_hessian);

  OptProblem& SetConvergeVar(ConvergeVarFn const& converge_var);

  OptProblem& SetConvergeGrad(ConvergeGradFn const& converge_grad);

  OptProblem& SetVerbose(VerboseFn const& verbose);

private:
  EnergyFn energy_{nullptr};
  GradFn grad_{nullptr};
  HessianFn hessian_{nullptr};
  SparseHessianFn sparse_hessian_{nullptr};
  ConvergeVarFn converge_var_{nullptr};
  ConvergeGradFn converge_grad_{nullptr};
  VerboseFn verbose_{nullptr};
};

/****************************** Optimization Result ******************************/
struct OptResultImpl {
  // Shared Result:
  math::vecxr x_opt_;
  real f_opt_;

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
