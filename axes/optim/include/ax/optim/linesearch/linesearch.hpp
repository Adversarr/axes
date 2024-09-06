#pragma once

#include "ax/math/functional.hpp"
#include "ax/optim/common.hpp"
#include "ax/optim/linesearch/common.hpp"  // IWYU pragma: export
#include "ax/utils/enum_refl.hpp"
#include "ax/utils/opt.hpp"

namespace ax::optim {

AX_DEFINE_ENUM_CLASS(LineSearchKind, Backtracking, Wolfe, Exact, Null);

class LinesearchBase : public utils::Tunable {
public:
  virtual ~LinesearchBase() = default;

  virtual OptResult Optimize(OptProblem const& prob, Variable const& x0, Gradient const& grad,
                             Variable const& dir) const
      = 0;
  virtual LineSearchKind GetKind() const = 0;

  static std::unique_ptr<LinesearchBase> Create(LineSearchKind kind);

  utils::Options GetOptions() const override;

  void SetOptions(const utils::Options& option) override;

  Index max_iter_{100};
  Real min_step_size_{1e-5};
  Real max_step_size_{1e3};
  Real initial_step_size_{1.0};
  bool verbose_{false};

  Real tol_grad_{1e-6}; ///< tolerance for gradient norm
};

}  // namespace ax::optim
