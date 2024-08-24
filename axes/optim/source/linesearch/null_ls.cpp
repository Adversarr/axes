#include "ax/optim/linesearch/null_ls.hpp"

namespace ax::optim {

LineSearchKind Linesearch_Null::GetKind() const {
  return LineSearchKind::kNull;
}

OptResult Linesearch_Null::Optimize(const OptProblem& prob, const Variable& x0,
                                    const Gradient& grad, const Variable& dir) const {
  Variable x = x0 + initial_step_size_ * dir;
  Real energy = prob.EvalEnergy(x);

  return OptResult::ConvergedLinesearch(std::move(x), energy, 0, initial_step_size_);
}

}  // namespace ax::optim
