#include "ax/xpbd/common.hpp"

#include "ax/core/entt.hpp"
#include "ax/utils/iota.hpp"
namespace ax::xpbd {

template <idx dim> void ConstraintBase<dim>::OnAttach() const {}

template <idx dim> void ConstraintBase<dim>::OnDetach() const {}

template <idx dim> UPtr<ConstraintBase<dim>> ConstraintBase<dim>::Create(ConstraintKind kind) {
  switch (kind) {
    // TODO: Implement these classes
    // case ConstraintKind::kInertia: return std::make_unique<InertiaConstraint<dim>>();
    // case ConstraintKind::kSpring: return std::make_unique<SpringConstraint<dim>>();
    // case ConstraintKind::kCollision: return std::make_unique<CollisionConstraint<dim>>();
    // case ConstraintKind::kHard: return std::make_unique<HardConstraint<dim>>();
    default:
      return nullptr;
  }
}

template <idx dim> void ConsensusAdmmSolver<dim>::BeginSimulation() {
  auto &g = ensure_server<dim>();
  for (auto& c : g.constraints_) {
    c->OnAttach();
  }
}

template <idx dim> void ConstraintBase<dim>::UpdatePositionConsensus() {
  idx n_v = this->GetNumConstrainedVertices();
  auto const& cmap = this->constraint_mapping_;
  math::fieldr<dim> fetch_from_global(dim, n_v);
  auto const& g = ensure_server<dim>();
  for (idx i : utils::iota(n_v)) {
    idx iV = cmap(0, i);
    fetch_from_global.col(i) = g.vertices_.col(iV);
  }
}

template <> GlobalServer<2>& ensure_server<2>() {
  return ax::ensure_resource<GlobalServer<2>>();
}

template <> GlobalServer<3>& ensure_server<3>() {
  return ax::ensure_resource<GlobalServer<3>>();
}

template class ConstraintBase<2>;
template class ConstraintBase<3>;
}  // namespace ax::xpbd