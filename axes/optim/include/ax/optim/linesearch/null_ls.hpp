#pragma once
#include "linesearch.hpp"
namespace ax::optim {

class Linesearch_Null final : public LinesearchBase {
public:
  LineSearchKind GetKind() const override;

  OptResult Optimize(const OptProblem &prob, const Variable &x0, const Gradient &grad,
                     const Variable &dir) const override;
};

}  // namespace ax::optim
