#include "ax/optim2/optimizer/base.hpp"
#include "ax/optim2/linesearch/base.hpp"

namespace ax::optim2 {

OptimizerBase::OptimizerBase() = default;

OptimizerBase::~OptimizerBase() = default;

void OptimizerBase::SetLinesearch(LineSearchPtr ls) {
  linesearch_ = std::move(ls);
}
}

