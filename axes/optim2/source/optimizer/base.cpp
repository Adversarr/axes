#include "ax/optim2/optimizer/base.hpp"

#include "ax/optim2/linesearch/base.hpp"

namespace ax::optim2 {

OptimizerBase::OptimizerBase() = default;

OptimizerBase::~OptimizerBase() = default;

void OptimizerBase::SetLinesearch(LineSearchPtr ls) {
  linesearch_ = std::move(ls);
}

bool OptimizerBase::TestCriteria(const OptimizeParam& param, OptimizeResult& result) {
  bool any_converged = false;
  if (problem_->GetGaridentNorm() < param.tol_grad_.value_or(tol_grad_)) {
    result.converged_grad_ = true;
    any_converged = true;
  }
  if (result.n_iter_ > 0 && problem_->GetLastVariableChange() < param.tol_var_.value_or(tol_var_)) {
    result.converged_var_ = true;
    any_converged = true;
  }
  return any_converged;
}

}  // namespace ax::optim2
