#pragma once
#include "ax/core/gsl.hpp"
#include "ax/optim2/common.hpp"
#include "ax/utils/opt.hpp"

namespace ax::optim2 {

AX_DEFINE_ENUM_CLASS(OptimizerKind, Newton, GradientDescent, Lbfgs, Fista, NonlinearCg);

class OptimizerBase : public utils::Tunable {
public:
  OptimizerBase() = default;
  ~OptimizerBase() override = default;

  void SetProblem(shared_not_null<ProblemBase> problem) { problem_ = problem; }

  void SetLinesearch(LineSearchPtr ls) { linesearch_ = std::move(ls); }

  virtual void Optimize() const = 0;

  virtual OptimizerKind GetKind() const = 0;

  ///// public members: we do not need use trival getter and setter here.

  // These parameters could be override by input parameters.
  size_t max_iter_{1000};  // Maximum iteration.
  Real tol_grad_{1e-6};    // The tolerance for gradient norm.
  Real tol_var_{1e-12};    // The tolerance for variable change.
  bool verbose_{false};    // Print verbose information.

protected:
  LineSearchPtr linesearch_;
  ProblemPtr problem_;
};

}  // namespace ax::optim2