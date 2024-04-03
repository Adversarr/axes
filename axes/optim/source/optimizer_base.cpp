#include "ax/optim/optimizer_base.hpp"

#include "ax/core/echo.hpp"

namespace ax::optim {

void OptimizerBase::SetMaxIter(idx max_iter) { max_iter_ = max_iter; }

void OptimizerBase::SetTolVar(real tol_var) { tol_var_ = tol_var; }

void OptimizerBase::SetTolGrad(real tol_grad) { tol_grad_ = tol_grad; }

Status OptimizerBase::SetOptions(utils::Opt const& options) {
  if (options.Has<idx>("max_iter")) {
    AX_CHECK(options.Holds<idx>("max_iter")) << "max_iter must be an integer";
    SetMaxIter(options.Get<idx>("max_iter"));
  }
  if (options.Has<real>("tol_var")) {
    AX_CHECK(options.Holds<real>("tol_var")) << "tol_var must be a real number";
    SetTolVar(options.Get<real>("tol_var"));
  }
  if (options.Has<real>("tol_grad")) {
    AX_CHECK(options.Holds<real>("tol_grad")) << "tol_grad must be a real number";
    SetTolGrad(options.Get<real>("tol_grad"));
  }
  if (options.Has<idx>("verbose")) {
    AX_CHECK(options.Holds<idx>("verbose")) << "verbose must be a boolean";
    verbose_ = options.Get<idx>("verbose") != 0;
  }
  AX_RETURN_OK();
}

utils::Opt OptimizerBase::GetOptions() const {
  utils::Opt options{
    {"max_iter", max_iter_},
    {"tol_var", tol_var_},
    {"tol_grad", tol_grad_},
    {"verbose", idx(verbose_ ? 1 : 0)}
  };
  return options;
}

}  // namespace ax::optim
