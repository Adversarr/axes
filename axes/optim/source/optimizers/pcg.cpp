#include "ax/optim/optimizers/pcg.hpp"

#include "ax/core/entt.hpp"

namespace ax::optim {

Optimizer_NonlinearCg::Optimizer_NonlinearCg() = default;

Optimizer_NonlinearCg::~Optimizer_NonlinearCg() = default;


OptimizerKind Optimizer_NonlinearCg::GetKind() const {
  return OptimizerKind::kNonlinearCg;
}

utils::Options Optimizer_NonlinearCg::GetOptions() const {
  utils::Options options;
  options["strategy"] = utils::reflect_name(strategy_).value_or("???");
  return options;
}

} // namespace ax::optim