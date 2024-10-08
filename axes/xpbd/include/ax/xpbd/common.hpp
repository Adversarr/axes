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

AX_DEFINE_ENUM_CLASS(ConstraintKind, Inertia,
                  Spring,  // the most common elasticity term.
                  Tetra,   // FEM-like energy.
                  Collision,
                  PlaneCollider,
                  BallCollider,
                  VertexFaceCollider,
                  EdgeEdgeCollider,
                  CollidingBalls,
                  Hard);

struct ConstraintSolution {
  math::RealField3 weighted_position_;
  math::RealField1 weights_;
  Real sqr_dual_residual_;

  ConstraintSolution(Index n_vert) : weighted_position_(3, n_vert), weights_(1, n_vert) {
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
  virtual Real UpdateDuality() = 0;

  virtual void UpdatePositionConsensus();

  virtual ~ConstraintBase() = default;

  virtual void OnAttach() const;
  virtual void OnDetach() const;

  virtual void UpdateRhoConsensus(Real scale);

  virtual void BeginStep() = 0;
  virtual void EndStep();

  Index GetNumConstrainedVertices() const { return constrained_vertices_ids_.size(); }
  Index GetNumConstraints() const { return constraint_mapping_.Entries().size(); }
  ConstraintMap const& GetConstraintMapping() const { return constraint_mapping_; }
  std::vector<Index> const& GetConstrainedVerticesIds() const { return constrained_vertices_ids_; }

protected:
  std::vector<Index> constrained_vertices_ids_;
  std::vector<math::RealVector3> constrained_vertices_position_;

  // rows=#v per constraint,
  // we always assume, to compute the dual variable, always [1] + [2] + ... + [n-1] - [n]
  // if rows=1, then we assume the dual is [1], just the identity of the only vertex
  ConstraintMap constraint_mapping_;  ///< Local constraint map, each index is local.
  std::vector<Real> rho_;                  ///< Local weighting.
  Real rho_global_;                  ///< Global weighting.
  Real primal_tolerance_{1e-7};      ///< Tolerance for primal
};

class GlobalServer : utils::Tunable {
public:
  GlobalServer() = default;
  GlobalServer(GlobalServer&&) = default;
  ~GlobalServer() = default;

  // We only care about 1st order euler.
  math::RealField3 last_vertices_;
  math::RealField3 vertices_;
  math::RealField3 velocities_;
  math::RealField3 ext_accel_;
  math::RealField1 mass_;

  // For Collision Detectors.
  std::vector<math::IndexVector3> faces_;
  std::vector<math::IndexVector2> edges_;
  geo::BroadPhaseResult potential_collisions_;

  // High level meta.
  Real dt_;


  Real primal_dual_threshold_{10};  ///< Threshold for primal-dual convergence.
  Real dual_primal_threshold_{10};  ///< Threshold for dual-primal convergence.
  Real primal_dual_ratio_{1.5};     ///< Ratio for primal-dual, rho *= ratio.
  Real dual_primal_ratio_{1.5};     ///< Ratio for dual-primal, rho /= ratio.

  // Constraints.
  std::vector<std::unique_ptr<ConstraintBase>> constraints_;
};

GlobalServer& ensure_server();

}  // namespace ax::xpbd
