#include "ax/optim/linesearch/backtracking.hpp"
#include "ax/optim/linesearch/wolfe.hpp"

namespace ax::optim {

UPtr<LinesearchBase> LinesearchBase::Create(LineSearchKind kind) {
  switch (kind) {
    case LineSearchKind::kBacktracking:
      return std::make_unique<Linesearch_Backtracking>();
    case LineSearchKind::kWolfe:
      return std::make_unique<Linesearch_Wofle>();
    default:
      return nullptr;
  }
}

}  // namespace ax::optim
