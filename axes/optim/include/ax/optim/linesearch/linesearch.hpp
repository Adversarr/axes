#pragma once

#include "ax/optim/common.hpp"
#include "ax/utils/opt.hpp"
#include "ax/utils/enum_refl.hpp"

namespace ax::optim {

BOOST_DEFINE_ENUM_CLASS(LineSearchKind,
  kBacktracking,
  kWofle,
  kAjimo);

class LinesearchBase : public utils::Tunable {
public:
  virtual ~LinesearchBase() = default;

  virtual OptResult Optimize(OptProblem const& prob, math::vecxr const& x0, math::vecxr const& grad, math::vecxr const& dir) const = 0;

  static UPtr<LinesearchBase> Create(LineSearchKind kind);

protected:
  idx max_iter_{100};
};

}  // namespace ax::optim
