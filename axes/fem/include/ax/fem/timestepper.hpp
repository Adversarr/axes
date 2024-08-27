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
template <int dim>
class TimeStepperBase : public utils::Tunable {
public:
  // Constructors and destructors
  TimeStepperBase(std::shared_ptr<TriMesh<dim>> mesh);
  AX_DECLARE_CONSTRUCTOR(TimeStepperBase, delete, default);
  virtual ~TimeStepperBase();

  // Common Data Accessors
  std::shared_ptr<TriMesh<dim>> GetMesh() {
    return mesh_;
  }

  ElasticityComputeBase<dim> &GetElasticity() {
    return *elasticity_;
  }

  TimestepSchemeBase<dim> &GetIntegrationScheme() {
    return *integration_scheme_;
  }

  virtual void SetOptions(const utils::Options &option) override;
  virtual utils::Options GetOptions() const override;

  // External force getter/setter
  math::RealField<dim> const &GetExternalAcceleration() const {
    return ext_accel_;
  }

  void SetExternalAcceleration(math::RealField<dim> const &ext_force) {
    ext_accel_ = ext_force;
  }

  void SetExternalAccelerationUniform(math::RealVector<dim> const &ext_force) {
    SetExternalAcceleration(ext_force.replicate(1, mesh_->GetNumVertices()));
  }

  // Do the time stepping.
  /*************************
   * SECT: Constant APIs.
   *************************/
  math::RealField<dim> const &GetDisplacement() const {
    return u_;
  }

  math::RealField<dim> const &GetLastDisplacement() const {
    return u_back_;
  }

  math::RealField<dim> const &GetNextDisplacementDelta() const {
    return du_;
  }

  math::RealField<dim> const &GetVelocity() const {
    return velocity_;
  }

  math::RealField<dim> GetPosition() const;
  math::RealField<dim> GetLastPosition() const;

  // SECT: Density: Will update the mass matrices.
  void SetDensity(Real density);
  void SetDensity(math::RealField1 const &density);

  math::RealSparseMatrix const &GetMassMatrix() const noexcept {
    return mass_matrix_;
  }

  math::RealSparseMatrix const &GetMassMatrixOriginal() const noexcept {
    return mass_matrix_original_;
  }

  // SECT: Lame:
  void SetYoungs(Real youngs);
  void SetPoissonRatio(Real poisson_ratio);
  void SetLame(const math::RealField2 &lame);
  void SetLame(const math::RealVector2 &u_lame);

  math::RealField2 const &GetLame() const noexcept {
    return lame_;
  }

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
  virtual void BeginSimulation(Real dt);
  // Start a new timestep, prepare the data, we assume dt does not change during this timestep
  virtual void BeginTimestep();
  // Set the mesh to new position
  virtual void EndTimestep();
  virtual void EndTimestep(math::RealField<dim> const &du);
  // Solve the timestep
  optim::OptProblem AssembleProblem();
  void RecomputeInitialGuess(math::RealField<dim> const &u, math::RealField<dim> const &u_back,
                             math::RealField<dim> const &velocity,
                             math::RealField<dim> const &velocity_back,
                             math::RealField<dim> const &ext_accel);

  math::RealField<dim> const &GetInitialGuess() const {
    return du_inertia_;
  }

  math::RealField<dim> const &GetSolution() const {
    return du_;
  }

  virtual void SolveTimestep();

  // SECT: During the timestep, optimizers may require such information

  Real Energy(math::RealField<dim> const &u) const;
  math::RealField<dim> GetElasticForce(math::RealField<dim> const &u) const;
  math::RealSparseMatrix GetStiffnessMatrix(math::RealField<dim> const &u,
                                            bool project = false) const;
  math::RealField<dim> Gradient(math::RealField<dim> const &u) const;
  math::RealVectorX GradientFlat(math::RealVectorX const &u_flat) const;
  math::RealSparseMatrix Hessian(math::RealField<dim> const &u, bool project = true) const;

  Real ResidualNorm(math::RealField<dim> const &grad) const;
  Real L2Residual(math::RealField<dim> const &grad) const;
  Real L1Residual(math::RealField<dim> const &grad) const;
  Real LinfResidual(math::RealField<dim> const &grad) const;

  std::vector<math::RealField<dim>> const &GetLastTrajectory() const {
    return last_trajectory_;
  }

  std::vector<Real> const &GetLastEnergy() const {
    return last_energy_;
  }

protected:
  /************************* SECT: Common Data *************************/
  std::shared_ptr<TriMesh<dim>> mesh_;
  std::shared_ptr<ElasticityComputeBase<dim>> elasticity_;
  std::unique_ptr<TimestepSchemeBase<dim>> integration_scheme_;

  // This is a little bit tricky, we need to store the name of the elasticity model
  std::string elasticity_name_{"stable_neohookean"}, device_{"cpu"};

  // NOTE: Elasticity.
  Real youngs_{1e7}, poisson_ratio_{0.33}, density_{1e3};

  // State variables
  math::RealField<dim> u_, u_back_;                ///< Displacement
  math::RealField<dim> velocity_, velocity_back_;  ///< Velocity
  math::RealField<dim> ext_accel_;                 ///< External acceleration
  // TODO: we do not need these variables, we can move them to the integration scheme.

  /************************* SECT: Solver Results *************************/
  // NOTE: Stores the solution of displacement of displacement.
  math::RealField<dim> du_inertia_, du_, u_inertia_;

  // runtime parameters.
  math::RealField2 lame_;                        ///< Lame parameters
  math::RealSparseMatrix mass_matrix_;           ///< The full mass matrix, (N D, N D)
  math::RealSparseMatrix mass_matrix_original_;  ///< The original mass matrix, (N, N)

  Real dt_{1e-2};  ///< The time step, should be formulated here because many initializers use dt
                   ///< such as Quasi Newton proposed in Liu17.

  Real abs_tol_grad_{0};  // Dynamically set according to rel_tol_grad.

  // Convergency Parameters:
  Real rel_tol_grad_{0.02};
  Real tol_var_{1e-9};
  TimestepConvergeNormKind converge_kind_{TimestepConvergeNormKind::kLinf};
  Index max_iter_{1000};
  bool record_trajectory_{false};

  std::vector<math::RealField<dim>> last_trajectory_;
  std::vector<Real> last_energy_;
  Index last_iterations_{0};

private:
  bool has_time_step_begin_{false};
  bool has_initialized_{false};
  bool has_simulation_begun_{false};

  template <template <int> class ElasticModelTemplate,
            template <int, template <int> class> class Compute = ElasticityCompute_CPU>
  void SetupElasticity();
};

}  // namespace ax::fem
