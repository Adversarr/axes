#pragma once
#include "ax/utils/enum_refl.hpp"
#include "ax/utils/opt.hpp"
#include "common.hpp"

namespace ax::vdb {

BOOST_DEFINE_ENUM(PoissonSolverKind, kVdb);

class PoissonSolverBase : public utils::Tunable {
public:
  using BcFunc = std::function<void(Coord const&, Coord const&, Real&, Real&)>;

  virtual RealGridPtr operator()(RealGridPtr source) = 0;

  virtual ~PoissonSolverBase() = default;

  static std::unique_ptr<PoissonSolverBase> Create(PoissonSolverKind kind);

  void SetBoundaryCondition(BcFunc boundary_condition);
protected:
  BcFunc boundary_condition_;
};

}  // namespace ax::vdb
