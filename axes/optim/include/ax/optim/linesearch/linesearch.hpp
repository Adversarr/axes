#pragma once

#include "ax/optim/common.hpp"
#include "ax/utils/opt.hpp"

namespace ax::optim {

enum class LineSearchKind { kBacktracking , kAjimo };

class LinesearchBase : public utils::Tunable {
public:
  virtual ~LinesearchBase() = default;

  virtual OptResult Optimize(OptProblem const& prob, math::vecxr const& x0, math::vecxr const& dir) const = 0;

  static UPtr<LinesearchBase> Create(LineSearchKind kind);

protected:
  idx max_iter_{100};
};

}  // namespace ax::optim
#include "ax/utils/enum_refl.hpp"

AX_ENUM_REFL_BEGIN(ax::optim::LineSearchKind)
AX_ENUM_STATE(kBacktracking, Backtracking)
AX_ENUM_REFL_END();
