#pragma once
#include "ax/core/buffer/buffer_view.hpp"
#include "ax/math/common.hpp"

namespace ax::optim2 {

AX_DEFINE_ENUM_CLASS(LinesearchInterpolationKind, None, Quadratic, Cubic);

class ProblemBase;
class OptimizerBase;
class LineSearchBase;

//! use shared ptr to store the problem.
using ProblemPtr = std::shared_ptr<ProblemBase>;
using OptimizerPtr = std::unique_ptr<OptimizerBase>;
using LineSearchPtr = std::unique_ptr<LineSearchBase>;

struct LineSearchResult {
  Real f_opt_{math::nan<Real>};
  size_t n_iter_{0};
  Real step_length_{1.0};
  std::optional<bool> converged_strong_wolfe_;
  bool converged_;
  std::string err_msg_;
};

struct OptimizeResult {
  Real f_opt_{math::nan<Real>};
  size_t n_iter_{0};

  bool converged_grad_{false};  // Stronge convergence criteria.
  bool converged_var_{false};   // Weak convergence criteria.
};

struct OptimizeParam {
  std::optional<size_t> max_iter_;  ///< maximum iteration
  std::optional<Real> tol_grad_;    ///< tolerance for gradient norm
  std::optional<Real> tol_var_;     ///< tolerance for variable change
};

struct LineSearchParam {
  ConstRealBufferView search_direction_;

  // The following variables will overrides the optimizer internal option.

  // step size
  std::optional<Real> min_step_size_;
  std::optional<Real> max_step_size_;
  std::optional<Real> initial_step_size_;

  // criteria
  std::optional<Real> armijo_;
  std::optional<Real> curvature_;
  std::optional<Real> strong_wolfe_;

  // step shrink
  std::optional<Real> step_shrink_factor_;
  std::optional<Real> min_step_shrink_factor_;
  std::optional<Real> max_step_shrink_factor_;

  // step expand
  std::optional<Real> step_expand_factor_;
  std::optional<Real> max_step_expand_factor_;

  // interpolation to determine next step size.
  std::optional<LinesearchInterpolationKind> interpolation_kind_;

  // Maximum iteration.
  std::optional<size_t> max_iter_;

  explicit LineSearchParam(ConstRealBufferView search_direction)
      : search_direction_(search_direction) {}
};

}  // namespace ax::optim2