/**
 * @brief Mass spring is also an inexact method.
 *
 */

#pragma once
#include "ax/core/common.hpp"
#include "ax/core/config.hpp"
#include "ax/math/common.hpp"
#include "ax/utils/opt.hpp"

namespace ax::xpbd {
template <idx dim> class GlobalServer;

BOOST_DEFINE_ENUM(ConstraintKind, kInertia,
                  kSpring,  // the most common elasticity term.
                  kCollision, kHard);

template <idx dim> struct ConstraintSolution {
  math::fieldr<dim> weighted_position_;
  math::field1r weights_;

  ConstraintSolution(idx n_vert) : weighted_position_(dim, n_vert), weights_(n_vert) {
    weighted_position_.setZero();
    weights_.setZero();
  }
};

/**
 * @brief Constraints in Consensus ADMM.
 * Solve Distributed:
 * --> x_i [k+1] := argmin_(x_i) ( f_i(x_i) + y_i[k].T xi + (ρ/2)‖xi − z˜[k] i ‖2 2 )
 * Update Consensus:
 * --> z˜i [k+1] := argmin_(z)   ( ∑ ( −y_i[k].T i z˜i + (ρ/2)‖xk+1 i − z˜i‖2 2 ))
 * Update Duality step:
 * --> y_i [k+1] := y_i[k] + ρ(x_i[k+1] − z_i[k+1])
 *
 * @tparam dim
 */
template <idx dim> class ConstraintBase : utils::Tunable {
public:
  static UPtr<ConstraintBase<dim>> Create(ConstraintKind kind);
  virtual ConstraintKind GetKind() const = 0;

  // Update dual variable
  virtual ConstraintSolution<dim> SolveDistributed() = 0;
  virtual void UpdateDuality() = 0;

  void UpdatePositionConsensus();

  virtual ~ConstraintBase() = default;

  virtual void OnAttach() const;
  virtual void OnDetach() const;

  virtual void BeginStep() = 0;
  virtual void EndStep() = 0;

  idx GetNumVerticesPerConstraint() const { return constrained_vertices_ids_.rows(); }
  idx GetNumConstrainedVertices() const { return constrained_vertices_ids_.cols(); }
  idx GetNumConstraints() const { return constraint_mapping_.cols(); }
  math::matxxi const& GetConstraintMapping() const { return constraint_mapping_; }
  math::field1i const& GetConstrainedVerticesIds() const { return constrained_vertices_ids_; }

protected:
  math::field1i constrained_vertices_ids_;
  math::fieldr<dim> constrained_vertices_position_;

  // rows=#v per constraint,
  // we always assume, to compute the dual variable, always [1] + [2] + ... + [n-1] - [n]
  // if rows=1, then we assume the dual is [1], just the identity of the only vertex
  math::matxxi constraint_mapping_;

  real rho_{1.0};
};

template <idx dim> class ConsensusAdmmSolver : utils::Tunable {
public:
  void BeginSimulation();
  void BeginTimestep();
  void SolveTimestep();
  void EndTimestep();

private:
  idx max_iter_;
  real dt_;
  real rho_;
};

template <idx dim> class GlobalServer {
public:
  // We only care about 1st order euler.
  math::fieldr<dim> vertices_;
  math::fieldr<dim> velocities_;
  math::fieldr<dim> ext_forces_;

  math::fieldr<dim> inertia_positions_;

  // Constraints.
  List<UPtr<ConstraintBase<dim>>> constraints_;
};

template <idx dim> GlobalServer<dim>& ensure_server();

}  // namespace ax::xpbd
