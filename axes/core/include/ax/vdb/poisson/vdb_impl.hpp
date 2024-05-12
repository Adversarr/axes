#include "ax/vdb/poisson.hpp"

namespace ax::vdb {

class VdbPoissonSolver : public PoissonSolverBase {
public:
  VdbPoissonSolver() = default;
  ~VdbPoissonSolver() = default;
  StatusOr<RealGridPtr> operator()(RealGridPtr source) override;

  virtual void SetOptions(utils::Opt const& option) final;
  virtual utils::Opt GetOptions() const final;

private:
  idx max_iterations_ = 50;
  real rel_error_ = 1.0e-6;
  real abs_error_ = std::numeric_limits<real>::epsilon() * 100.0;
};

}