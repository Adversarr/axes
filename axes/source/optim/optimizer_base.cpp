#include "axes/optim/optimizer_base.hpp"

namespace ax::optim {

void OptimizerBase::SetMaxIter(idx max_iter) { max_iter_ = max_iter; }

void OptimizerBase::SetTolVar(real tol_var) { tol_var_ = tol_var; }

void OptimizerBase::SetTolGrad(real tol_grad) { tol_grad_ = tol_grad; }

}  // namespace ax::optim
