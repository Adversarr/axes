#pragma once
#include "linesearch.hpp"

namespace ax::optim {

/// Implements Golden-section search for exact line search.
class Linesearch_Exact final : public LinesearchBase {
public:
  Linesearch_Exact() = default;
  ~Linesearch_Exact() override = default;

  LineSearchKind GetKind() const override;

  OptResult Optimize(OptProblem const& prob, Variable const& x0, Gradient const& grad,
                     Variable const& dir) const override;

  utils::Options GetOptions() const override;

  void SetOptions(utils::Options const& option) override;

private:
  Real initial_step_size_ = 1.0;
  Real required_accuracy_ = 1e-2;  ///< Stopping Criteria: |x_{k+1} - x_k| < required_accuracy_
};

}  // namespace ax::optim
