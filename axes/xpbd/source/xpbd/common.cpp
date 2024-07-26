#include "ax/xpbd/common.hpp"

#include <memory>

#include "ax/core/entt.hpp"
#include "ax/utils/iota.hpp"
#include "ax/xpbd/constraints/ball_collider.hpp"
#include "ax/xpbd/constraints/colliding_balls.hpp"
#include "ax/xpbd/constraints/edge_edge_collider.hpp"
#include "ax/xpbd/constraints/hard.hpp"
#include "ax/xpbd/constraints/inertia.hpp"
#include "ax/xpbd/constraints/plane_collider.hpp"
#include "ax/xpbd/constraints/spring.hpp"
#include "ax/xpbd/constraints/tet.hpp"
#include "ax/xpbd/constraints/vertex_face_collider.hpp"

namespace ax::xpbd {

void ConstraintBase::OnAttach() const {}
void ConstraintBase::OnDetach() const {}

std::unique_ptr<ConstraintBase> ConstraintBase::Create(ConstraintKind kind) {
  switch (kind) {
    // TODO: Implement these classes
    case ConstraintKind::kInertia:
      return std::make_unique<Constraint_Inertia>();
    case ConstraintKind::kSpring:
      return std::make_unique<Constraint_Spring>();
    case ConstraintKind::kTetra:
      return std::make_unique<Constraint_Tetra>();
    case ConstraintKind::kPlaneCollider:
      return std::make_unique<Constraint_PlaneCollider>();
    case ConstraintKind::kHard:
      return std::make_unique<Constraint_Hard>();
    case ConstraintKind::kBallCollider:
      return std::make_unique<Constraint_BallCollider>();
    case ConstraintKind::kVertexFaceCollider:
      return std::make_unique<Constraint_VertexFaceCollider>();
    case ConstraintKind::kEdgeEdgeCollider:
      return std::make_unique<Constraint_EdgeEdgeCollider>();
    case ConstraintKind::kCollidingBalls:
      return std::make_unique<Constraint_CollidingBalls>();
    default:
      return nullptr;
  }
}

void ConstraintBase::UpdatePositionConsensus() {
  idx n_v = this->GetNumConstrainedVertices();
  auto const& cmap = this->constrained_vertices_ids_;
  auto& local = constrained_vertices_position_;
  local.resize(n_v);
  auto const& g = ensure_server();
  for (idx i : utils::iota(n_v)) {
    idx iV = cmap[i];
    local[i] = g.vertices_.col(iV);
  }
}

void ConstraintBase::EndStep() {}
void ConstraintBase::UpdateRhoConsensus(real) {}

GlobalServer& ensure_server() { return ax::ensure_resource<GlobalServer>(); }

}  // namespace ax::xpbd
