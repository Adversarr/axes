#include "ax/xpbd/common.hpp"

#include <memory>

#include "ax/core/entt.hpp"
#include "ax/utils/iota.hpp"
#include "ax/xpbd/constraints/ball_collider.hpp"
#include "ax/xpbd/constraints/hard.hpp"
#include "ax/xpbd/constraints/inertia.hpp"
#include "ax/xpbd/constraints/plane_collider.hpp"
#include "ax/xpbd/constraints/spring.hpp"
#include "ax/xpbd/constraints/tet.hpp"

namespace ax::xpbd {

template <idx dim> void ConstraintBase<dim>::OnAttach() const {}

template <idx dim> void ConstraintBase<dim>::OnDetach() const {}

template <idx dim> UPtr<ConstraintBase<dim>> ConstraintBase<dim>::Create(ConstraintKind kind) {
  switch (kind) {
    // TODO: Implement these classes
    case ConstraintKind::kInertia:
      return std::make_unique<Constraint_Inertia<dim>>();
    case ConstraintKind::kSpring:
      return std::make_unique<Constraint_Spring<dim>>();
    case ConstraintKind::kTetra:
      return std::make_unique<Constraint_Tetra<dim>>();
    case ConstraintKind::kPlaneCollider:
      return std::make_unique<Constraint_PlaneCollider<dim>>();
    case ConstraintKind::kHard:
      return std::make_unique<Constraint_Hard<dim>>();
    case ConstraintKind::kBallCollider:
      return std::make_unique<Constraint_BallCollider<dim>>();
    default:
      return nullptr;
  }
}

template <idx dim> void ConsensusAdmmSolver<dim>::BeginSimulation() {
  auto& g = ensure_server<dim>();
  for (auto& c : g.constraints_) {
    c->OnAttach();
  }
  if (g.velocities_.size() == 0) {
    g.velocities_.setZero(dim, g.velocities_.cols());
  }
  if (g.ext_forces_.size() == 0) {
    g.velocities_.setZero(dim, g.velocities_.cols());
  }
}

template <idx dim> void ConstraintBase<dim>::UpdatePositionConsensus() {
  idx n_v = this->GetNumConstrainedVertices();
  auto const& cmap = this->constrained_vertices_ids_;
  math::fieldr<dim>& local = constrained_vertices_position_;
  local.resize(dim, n_v);
  auto const& g = ensure_server<dim>();
  for (idx i : utils::iota(n_v)) {
    idx iV = cmap[i];
    local.col(i) = g.vertices_.col(iV);
  }
}

template <idx dim> void ConstraintBase<dim>::UpdateRhoConsensus(real) {}

template <> GlobalServer<2>& ensure_server<2>() { return ax::ensure_resource<GlobalServer<2>>(); }

template <> GlobalServer<3>& ensure_server<3>() { return ax::ensure_resource<GlobalServer<3>>(); }

template class ConstraintBase<2>;
template class ConstraintBase<3>;
}  // namespace ax::xpbd