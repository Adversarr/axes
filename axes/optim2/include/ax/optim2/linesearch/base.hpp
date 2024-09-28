#pragma once
#include "ax/optim2/problem.hpp"
#include "ax/utils/opt.hpp"

namespace ax::optim2 {

AX_DEFINE_ENUM_CLASS(LineSearchKind, Backtracking, Wolfe, Exact, Null);

class LineSearchBase : public utils::Tunable {
public:
  LineSearchBase() = default;
  virtual ~LineSearchBase() = default;

  void SetProblem(shared_not_null<ProblemBase> problem);

  /**
   * @brief Performs the line search optimization.
   *
   * @note If converged, the problem.var will be updated to the optimal.
   *
   * @param param
   * @return LineSearchResult
   */
  virtual LineSearchResult Optimize(LineSearchParam param) = 0;

  virtual LineSearchKind GetKind() const = 0;

  ///// public members: we do not need use trival getter and setter here.

  // step size
  Real min_step_size_{1e-6};
  Real max_step_size_{1e3};
  Real initial_step_size_{1};

  // criteria
  Real armijo_{1e-4};       // i.e. the expect descent rate.
  Real curvature_{0.9};     // i.e. the expect curvature rate.
  Real strong_wolfe_{0.9};  // i.e. the expect strong wolfe rate.

  // step shrink
  Real step_shrink_factor_{0.6};
  Real min_step_shrink_factor_{0.25};
  Real max_step_shrink_factor_{0.75};

  // step expand
  Real step_expand_factor_{1.5};
  Real max_step_expand_factor_{10.0};

  size_t max_iter_{100};  // Maximum iteration.

  // interpolation is used to determine next step size.
  LinesearchInterpolationKind interpolation_kind_{LinesearchInterpolationKind::Quadratic};

  Real OriginalGradientDotDirection() const;

  utils::Options GetOptions() const override;

  void SetOptions(const utils::Options &option) override;

protected:
  // return the current step size.
  Real GetCurrentStep() const;

  // set the problem.var to x0 + step * dir, and update the energy.
  // and gradient if needed.
  void StepTo(const LineSearchParam& param, Real step_size, bool update_gradient, size_t iter);

  // get the cached energy.
  Real CurrentEnergy() const;

  // the backuped variables, i.e. step size=0.
  Real OriginalEnergy() const;
  ConstRealBufferView OriginalGradient() const;
  ConstRealBufferView OriginalVariables() const;

  // For Derived class, should not access the underlying problem directly.
  void BeginSearch(LineSearchParam& param);
  void EndSearch(LineSearchResult& result);

  // Test the criteria for the current step size.
  bool TestCurrentArmojo(const LineSearchParam& param) const;
  bool TestCurrentCurvature(const LineSearchParam& param) const;
  bool TestCurrentStrongWolfe(const LineSearchParam& param) const;

  // Help you to fit the quadratic and get the optimal step size.
  Real SolveOptimalStepSizeQuadratic(const LineSearchParam& param, Real f_lo, Real g_lo, Real f_hi,
                                     Real lo, Real hi) const;
  Real SolveOptimalStepSizeNone(const LineSearchParam& param, Real lo, Real hi) const;

  void UpdateGradient();
  ConstRealBufferView CurrentGradient() const;

private:
  void FixParameter(LineSearchParam& param) const;
  Real current_step_size_;

  Real grad_dot_dir_x0_;   // = <g, d> at x0
  ProblemPtr problem_;
};

}  // namespace ax::optim2