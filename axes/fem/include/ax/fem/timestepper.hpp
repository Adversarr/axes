#pragma once
#include "ax/fem/elasticity/base.hpp"
#include "ax/math/common.hpp"
#include "ax/optim/common.hpp"
#include "ax/utils/enum_refl.hpp"
#include "ax/utils/opt.hpp"
#include "elasticity_cpu.hpp"
#include "scheme.hpp"
#include "trimesh.hpp"

namespace ax::fem {

AX_DEFINE_ENUM_CLASS(TimestepConvergeNormKind, kL2, kL1, kLinf);

// Use Backward Euler (with exactly 1 step backward)
//     M (x' - (x + dt v)) = dt^2 F_int(x')
// Apply 2nd Taylor expansion to approximate F_int(x') = F_int(x) + K (x' - x) + O(dx^2)
//    (M + dt^2 K) x' = M (x + dt v) + dt^2 F_int(x)
// where velocity = (x - x_{n-1})/dt
// This gives us a linear system to solve:
//    (M + dt^2 K) (x' - x) = dt M v + dt^2 F_int(x)
// Because we use Linear model, K is constant, and the approximation is exact.
// One step of Linear solve should give you the correct result.
// WARNING: You should not apply the base solver to solve real world problems, this implementation
//          is only reserved for testing.
template <int dim> class TimeStepperBase : public utils::Tunable {
public:
  // Constructors and destructors
  TimeStepperBase(std::shared_ptr<TriMesh<dim>> mesh);
  AX_DECLARE_CONSTRUCTOR(TimeStepperBase, delete, default);
  virtual ~TimeStepperBase() = default;

  // Common Data Accessors
  std::shared_ptr<TriMesh<dim>> GetMesh() { return mesh_; }
  ElasticityComputeBase<dim> &GetElasticity() { return *elasticity_; }
  TimestepSchemeBase<dim> &GetIntegrationScheme() { return *integration_scheme_; }
  virtual void SetOptions(const utils::Options &option) override;
  virtual utils::Options GetOptions() const override;

  // External force getter/setter
  math::fieldr<dim> const &GetExternalAcceleration() const { return ext_accel_; }
  void SetExternalAcceleration(math::fieldr<dim> const &ext_force) { ext_accel_ = ext_force; }
  void SetExternalAccelerationUniform(math::vecr<dim> const &ext_force) {
    SetExternalAcceleration(ext_force.replicate(1, mesh_->GetNumVertices()));
  }

  // Do the time stepping.
  /*************************
   * SECT: Constant APIs.
   *************************/
  math::fieldr<dim> const &GetDisplacement() const { return u_; }
  math::fieldr<dim> const &GetLastDisplacement() const { return u_back_; }
  math::fieldr<dim> const &GetNextDisplacementDelta() const { return du_; }
  math::fieldr<dim> const &GetVelocity() const { return velocity_; }
  math::fieldr<dim> GetPosition() const;
  math::fieldr<dim> GetLastPosition() const;

  // SECT: Density: Will update the mass matrices.
  void SetDensity(real density);
  void SetDensity(math::field1r const &density);
  math::spmatr const &GetMassMatrix() const noexcept { return mass_matrix_; }
  math::spmatr const &GetMassMatrixOriginal() const noexcept { return mass_matrix_original_; }

  // SECT: Lame:
  void SetYoungs(real youngs);
  void SetPoissonRatio(real poisson_ratio);
  void SetLame(const math::field2r &lame);
  void SetLame(const math::vec2r &u_lame);
  math::field2r const &GetLame() const noexcept { return lame_; }

  void SetupElasticity(std::string name, std::string device);

  /*************************
   * SECT: Runtime (new)
   *************************/
  // Requiest the Simulator to setup basic buffers. such as deformation map, velocity.
  // WARN: After this function is called, the simulation will be considered initialized:
  // The mesh cannot change, because we use the mesh to initialize the solution buffers.
  virtual void Initialize();
  // acknowledge the beginning of simulation

  // WARN: After this function is called, the simulation will be considered started:
  // Elasticity, integration scheme, and youngs, poisson_ratio, density cannot change
  // to avoid inconsistency.
  virtual void BeginSimulation(real dt);
  // Start a new timestep, prepare the data, we assume dt does not change during this timestep
  virtual void BeginTimestep();
  // Set the mesh to new position
  virtual void EndTimestep();
  virtual void EndTimestep(math::fieldr<dim> const &du);
  // Solve the timestep
  optim::OptProblem AssembleProblem();
  void RecomputeInitialGuess(math::fieldr<dim> const &u, math::fieldr<dim> const &u_back,
                             math::fieldr<dim> const &velocity, math::fieldr<dim> const &velocity_back,
                             math::fieldr<dim> const &ext_accel);
  math::fieldr<dim> const &GetInitialGuess() const { return du_inertia_; }
  math::fieldr<dim> const &GetSolution() const { return du_; }
  virtual void SolveTimestep();

  // SECT: During the timestep, optimizers may require such information

  real Energy(math::fieldr<dim> const &u) const;
  math::fieldr<dim> GetElasticForce(math::fieldr<dim> const &u) const;
  math::spmatr GetStiffnessMatrix(math::fieldr<dim> const &u, bool project = false) const;
  math::fieldr<dim> Gradient(math::fieldr<dim> const &u) const;
  math::vecxr GradientFlat(math::vecxr const &u_flat) const;
  math::spmatr Hessian(math::fieldr<dim> const &u, bool project = true) const;

  real ResidualNorm(math::fieldr<dim> const &grad) const;
  real L2Residual(math::fieldr<dim> const &grad) const;
  real L1Residual(math::fieldr<dim> const &grad) const;
  real LinfResidual(math::fieldr<dim> const &grad) const;

  std::vector<math::fieldr<dim>> const &GetLastTrajectory() const { return last_trajectory_; }
  std::vector<real> const &GetLastEnergy() const { return last_energy_; }

protected:
  /************************* SECT: Common Data *************************/
  std::shared_ptr<TriMesh<dim>> mesh_;
  std::unique_ptr<ElasticityComputeBase<dim>> elasticity_;
  std::unique_ptr<TimestepSchemeBase<dim>> integration_scheme_;

  // This is a little bit tricky, we need to store the name of the elasticity model
  std::string elasticity_name_{"stable_neohookean"}, device_{"cpu"};

  // NOTE: Elasticity.
  real youngs_{1e7}, poisson_ratio_{0.33}, density_{1e3};

  // State variables
  math::fieldr<dim> u_, u_back_;                ///< Displacement
  math::fieldr<dim> velocity_, velocity_back_;  ///< Velocity
  math::fieldr<dim> ext_accel_;                 ///< External acceleration
  // TODO: we do not need these variables, we can move them to the integration scheme.

  /************************* SECT: Solver Results *************************/
  // NOTE: Stores the solution of displacement of displacement.
  math::fieldr<dim> du_inertia_, du_, u_inertia_;

  // runtime parameters.
  // math::vec2r u_lame_;                 ///< Uniform Lame parameters
  math::field2r lame_;                 ///< Lame parameters
  math::spmatr mass_matrix_;           ///< The full mass matrix, (N D, N D)
  math::spmatr mass_matrix_original_;  ///< The original mass matrix, (N, N)

  real dt_{1e-2};  ///< The time step, should be formulated here because many initializers use dt
                   ///< such as Quasi Newton proposed in Liu17.

  real abs_tol_grad_{0};  // Dynamically set according to rel_tol_grad.

  // Convergency Parameters:
  real rel_tol_grad_{0.02};
  real tol_var_{1e-9};
  TimestepConvergeNormKind converge_kind_{TimestepConvergeNormKind::kLinf};
  idx max_iter_{1000};
  bool record_trajectory_{false};

  std::vector<math::fieldr<dim>> last_trajectory_;
  std::vector<real> last_energy_;
  idx last_iterations_{0};

private:
  bool has_time_step_begin_{false};
  bool has_initialized_{false};
  bool has_simulation_begun_{false};

  template <template <idx> class ElasticModelTemplate,
            template <idx, template <idx> class> class Compute = ElasticityCompute_CPU>
  void SetupElasticity();
};

}  // namespace ax::fem
