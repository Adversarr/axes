/**
 * @brief Mass spring is also an inexact method.
 *
 */

#pragma once
#include "ax/core/common.hpp"
#include "ax/core/config.hpp"
#include "ax/geometry/hittables.hpp"
#include "ax/math/common.hpp"
#include "ax/utils/opt.hpp"
#include "ax/xpbd/constraint_map.hpp"

namespace ax::xpbd {

AX_DEFINE_ENUM_CLASS(ConstraintKind, kInertia,
                  kSpring,  // the most common elasticity term.
                  kTetra,   // FEM-like energy.
                  kCollision,
                  kPlaneCollider,
                  kBallCollider,
                  kVertexFaceCollider,
                  kEdgeEdgeCollider,
                  kCollidingBalls,
                  kHard);

struct ConstraintSolution {
  math::field3r weighted_position_;
  math::field1r weights_;
  real sqr_dual_residual_;

  ConstraintSolution(idx n_vert) : weighted_position_(3, n_vert), weights_(1, n_vert) {
    weighted_position_.setZero();
    weights_.setZero();
    sqr_dual_residual_ = 0;
  }
};

/**
 * @brief Constraints in Consensus ADMM.
 * Solve Distributed:
 * --> x_i [k+1] := argmin_(x_i) ( f_i(x_i) + (ρ/2)‖ xi − z˜[k] i + y_i[k] ‖2 2 )
 * Update Consensus:
 * --> z˜i [k+1] := argmin_(z)   ( ∑ ( (ρ/2)‖xk+1 i + yi − z˜i‖2 2 ))
 * Update Duality step:
 * --> y_i [k+1] := y_i[k] + x_i[k+1] − z_i[k+1]
 *
 * @tparam dim
 */
class ConstraintBase : utils::Tunable {
public:
  static std::unique_ptr<ConstraintBase> Create(ConstraintKind kind);
  virtual ConstraintKind GetKind() const = 0;

  // Update dual variable
  virtual ConstraintSolution SolveDistributed() = 0;
  virtual real UpdateDuality() = 0;

  virtual void UpdatePositionConsensus();

  virtual ~ConstraintBase() = default;

  virtual void OnAttach() const;
  virtual void OnDetach() const;

  virtual void UpdateRhoConsensus(real scale);

  virtual void BeginStep() = 0;
  virtual void EndStep();

  idx GetNumConstrainedVertices() const { return constrained_vertices_ids_.size(); }
  idx GetNumConstraints() const { return constraint_mapping_.Entries().size(); }
  ConstraintMap const& GetConstraintMapping() const { return constraint_mapping_; }
  std::vector<idx> const& GetConstrainedVerticesIds() const { return constrained_vertices_ids_; }

protected:
  std::vector<idx> constrained_vertices_ids_;
  std::vector<math::vec3r> constrained_vertices_position_;

  // rows=#v per constraint,
  // we always assume, to compute the dual variable, always [1] + [2] + ... + [n-1] - [n]
  // if rows=1, then we assume the dual is [1], just the identity of the only vertex
  ConstraintMap constraint_mapping_;  ///< Local constraint map, each index is local.
  std::vector<real> rho_;                  ///< Local weighting.
  real rho_global_;                  ///< Global weighting.
  real primal_tolerance_{1e-7};      ///< Tolerance for primal
};

class GlobalServer : utils::Tunable {
public:
  GlobalServer() = default;
  GlobalServer(GlobalServer&&) = default;
  ~GlobalServer() = default;

  // We only care about 1st order euler.
  math::field3r last_vertices_;
  math::field3r vertices_;
  math::field3r velocities_;
  math::field3r ext_accel_;
  math::field1r mass_;

  // For Collision Detectors.
  std::vector<math::vec3i> faces_;
  std::vector<math::vec2i> edges_;
  geo::BroadPhaseResult potential_collisions_;

  // High level meta.
  real dt_;


  real primal_dual_threshold_{10};  ///< Threshold for primal-dual convergence.
  real dual_primal_threshold_{10};  ///< Threshold for dual-primal convergence.
  real primal_dual_ratio_{1.5};     ///< Ratio for primal-dual, rho *= ratio.
  real dual_primal_ratio_{1.5};     ///< Ratio for dual-primal, rho /= ratio.

  // Constraints.
  std::vector<std::unique_ptr<ConstraintBase>> constraints_;
};

GlobalServer& ensure_server();

}  // namespace ax::xpbd
