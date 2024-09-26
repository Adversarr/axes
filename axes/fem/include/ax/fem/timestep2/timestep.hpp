#pragma once

#include "ax/core/gsl.hpp"
#include "ax/fem/mesh.hpp"
#include "ax/fem/problem.hpp"
#include "ax/fem/terms/elasticity.hpp"
#include "ax/fem/terms/mass.hpp"
#include "ax/fem/utils/prune_dbc.hpp"
#include "ax/optim2/problem.hpp"
#include "ax/utils/opt.hpp"

namespace ax::fem {

class TimeStepVariationalProblem;

/**
 * @brief Time step solver base.
 * @note The variational form is :
 *       argmin 1/2 |u - u_inertia|_M^2  + dt^2 E(u)
 *
 */
class TimeStepBase : public utils::Tunable {
public:
  explicit TimeStepBase(shared_not_null<Mesh> mesh);
  ~TimeStepBase() override = default;

  const Problem &GetProblem() const;
  Problem &GetProblem();

  // some timestepper need to compute some auxiliary terms.
  virtual void Compute();

  // The inertia part.
  not_null<MassTerm *> GetInertia();

  // The elastic part.
  not_null<ElasticityTerm *> GetElasticity();

  // Helper function to set the elasticity.
  void SetElasticity(ElasticityKind kind);

  // Helper function to set the lame.
  virtual void SetLame(ConstRealBufferView lame);
  void SetLame(const math::RealVector2 &lame);  // uniform version

  // Helper function to set the density.
  virtual void SetDensity(ConstRealBufferView density);
  void SetDensity(Real density);  // uniform version

  // Helper function to set the external force.
  virtual void SetExternalAcceleration(ConstRealBufferView ext_accel);
  void SetExternalAcceleration(const math::RealVector2 &ext_accel);  // uniform version
  void SetExternalAcceleration(const math::RealVector3 &ext_accel);  // uniform version

  // Helper function, to update the Problem's energy, gradient, and hessian.
  void UpdateEnergy();
  void UpdateGradient();
  void UpdateHessian();

  // Set the timestep, also affect the coefficient of the elastic term.
  virtual void SetTimeStep(Real dt);

  // Prepare the problem, e.g. u_inertia.
  void BeginStep();
  // Solve the step.
  void MarkCurrentSolutionUpdated();
  void UpdateCurrentSolution(ConstRealBufferView u_current);
  virtual void SolveStep();
  // Flush all the buffer.
  void EndStep();

  // Helper: perform a single step.
  void Step();

  PruneDirichletBc &GetPruneDirichletBc();

  void UpdatePruneDirichletBc();

  std::unique_ptr<TimeStepVariationalProblem> PrepareVariationalProblem();

  void SetRelativeTolerance(Real tol_rel_grad);

  void SetOptions(utils::Options const& option) override;

protected:
  BufferPtr<Real> velocity_;   ///< The velocity of each vertex
  BufferPtr<Real> u_back_;     ///< The last displacement of each vertex
  BufferPtr<Real> u_;          ///< The displacement of each vertex,
                               ///< should be the same as the problem_.state_.variable_
  BufferPtr<Real> ext_accel_;  ///< The external acceleration.
  BufferPtr<Real> temp_;       ///< A temporary buffer, shape is same as u_.

  Real dt_;                     ///< The timestep.
  Problem problem_;             ///< The timestep problem.
  shared_not_null<Mesh> mesh_;  ///< The mesh.

  PruneDirichletBc prune_dirichlet_bc_;  ///< The Dirichlet boundary condition pruner.

  Real tol_rel_grad_{3e-3};   ///< Relative convergence criteria
  Real tol_abs_grad_{1e-12};  ///< Absolute convergence criteria, computed from relative

  bool is_set_density_called_ {false};
  bool is_set_lame_called_ {false};
  bool is_set_time_step_called_ {false};

private:
  MassTerm *cache_inertia_{nullptr};           ///< Cache the inertia term.
  ElasticityTerm *cache_elasticity_{nullptr};  ///< Cache the elasticity term.
  friend class TimeStepVariationalProblem;
};

class TimeStepVariationalProblem : public optim2::ProblemBase {
public:
  explicit TimeStepVariationalProblem(not_null<TimeStepBase *> timestep);

  void UpdateEnergy() override;
  void UpdateGradient() override;
  void UpdateHessian() override;
  void MarkVariableChanged() override;

private:
  not_null<TimeStepBase *> timestep_;

  // We do not need to store the dirty bit because the timestep_ will handle it.
};

}  // namespace ax::fem