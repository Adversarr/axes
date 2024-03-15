#pragma once
#include "axes/utils/enum_refl.hpp"
#include "axes/utils/opt.hpp"
#include "common.hpp"

namespace ax::vdb {

enum class PoissonSolverKind : int {
  kVdb,
  // kMultigrid,
};

class PoissonSolverBase : public utils::Tunable {
public:
  using BcFunc = std::function<void(Coord const&, Coord const&, real&, real&)>;

  virtual StatusOr<RealGridPtr> operator()(RealGridPtr source) = 0;

  virtual ~PoissonSolverBase() = default;

  static utils::uptr<PoissonSolverBase> Create(PoissonSolverKind kind);

  void SetBoundaryCondition(BcFunc boundary_condition);
protected:
  BcFunc boundary_condition_;
};

}  // namespace ax::vdb

AX_ENUM_REFL_BEGIN(ax::vdb::PoissonSolverKind)
AX_ENUM_STATE(kVdb, Vdb)
// AX_ENUM_STATE(kMultigrid, Multigrid)
AX_ENUM_REFL_END();