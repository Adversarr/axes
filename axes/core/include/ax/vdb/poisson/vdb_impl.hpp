#include "ax/vdb/poisson.hpp"

namespace ax::vdb {

class VdbPoissonSolver : public PoissonSolverBase {
public:
  VdbPoissonSolver() = default;
  ~VdbPoissonSolver() = default;
  RealGridPtr operator()(RealGridPtr source) override;

  virtual void SetOptions(utils::Options const& option) final;
  virtual utils::Options GetOptions() const final;

private:
  idx max_iterations_ = 50;
  real rel_error_ = 1.0e-6;
  real abs_error_ = std::numeric_limits<real>::epsilon() * 100.0;
};

}
