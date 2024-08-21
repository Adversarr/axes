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
  Index max_iterations_ = 50;
  Real rel_error_ = 1.0e-6;
  Real abs_error_ = std::numeric_limits<Real>::epsilon() * 100.0;
};

}
