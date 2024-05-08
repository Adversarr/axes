#pragma once
#include "ax/core/excepts.hpp"
#include "ax/math/common.hpp"
#include "ax/optim/common.hpp"
#include "ax/utils/enum_refl.hpp"
#include "ax/utils/opt.hpp"
#include "elasticity.hpp"
#include "scheme.hpp"
#include "trimesh.hpp"

namespace ax::fem {

BOOST_DEFINE_ENUM_CLASS(TimestepConvergeNormKind, kL2, kL1, kLinf);

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
template <idx dim> class TimeStepperBase : public utils::Tunable {
public:
  // Constructors and destructors
  TimeStepperBase() = default;
  TimeStepperBase(SPtr<TriMesh<dim>> mesh);
  AX_DECLARE_CONSTRUCTOR(TimeStepperBase, default, default);
  virtual ~TimeStepperBase() = default;

  // Common Data Accessors
  TriMesh<dim> &GetMesh() { return *mesh_; }
  SPtr<TriMesh<dim>> &GetMeshPtr() { return mesh_; }
  ElasticityComputeBase<dim> &GetElasticity() { return *elasticity_; }
  TimestepSchemeBase<dim> &GetIntegrationScheme() { return *integration_scheme_; }
  virtual Status SetOptions(const utils::Opt &option) override;
  virtual utils::Opt GetOptions() const override;

  // External force getter/setter
  math::fieldr<dim> const &GetExternalAcceleration() const { return ext_accel_; }
  void SetExternalAcceleration(math::fieldr<dim> const &ext_force) { ext_accel_ = ext_force; }
  void SetExternalAccelerationUniform(math::vecr<dim> const &ext_force) { SetExternalAcceleration(ext_force.replicate(1, mesh_->GetNumVertices())); }

  // Do the time stepping.
  /*************************
   * SECT: Constant APIs.
   *************************/
  math::fieldr<dim> const &GetDisplacement() const { return u_; }
  math::fieldr<dim> const &GetNextDisplacementDelta() const { return du_; }
  math::fieldr<dim> const &GetVelocity() const { return velocity_; }
  math::fieldr<dim> GetPosition() const { return u_ + mesh_->GetVertices(); }

  // SECT: Density: Will update the mass matrices.
  void SetDensity(real density);
  void SetDensity(math::field1r const &density);
  math::sp_matxxr const &GetMassMatrix() const { return mass_matrix_; }
  math::sp_matxxr const &GetMassMatrixOriginal() const { return mass_matrix_original_; }

  // SECT: Lame:
  void SetLame(math::vec2r const &lame) { u_lame_ = lame; }
  void SetYoungs(real youngs) { youngs_ = youngs; }
  void SetPoissonRatio(real poisson_ratio) { poisson_ratio_ = poisson_ratio; }
  math::vec2r const &GetLame() const { return u_lame_; }

  void SetupElasticity(std::string name, std::string device);
  /*************************
   * SECT: Runtim APIs
   *************************/

  math::sp_matxxr GetStiffnessMatrix(math::fieldr<dim> const &x, bool project = false) const;
  math::fieldr<dim> GetElasticForce(math::fieldr<dim> const &x) const;

  // WARN: Deprecated APIs.
  math::fieldr<dim> GetInertiaPosition(real dt) const;
  virtual Status Precompute();

  /*************************
   * SECT: Runtime (new)
   *************************/

  // Requiest the Simulator to setup basic buffers. such as deformation map, velocity.
  virtual Status Initialize();
  // acknowledge the beginning of simulation
  virtual void BeginSimulation(real dt = -1);
  // Start a new timestep, prepare the data, we assume dt does not change during this timestep
  virtual void BeginTimestep(real dt);
  // Set the mesh to new position
  virtual void EndTimestep();
  virtual void EndTimestep(math::fieldr<dim> const& du);
  // Solve the timestep
  optim::OptProblem AssembleProblem() const;
  math::fieldr<dim> const &GetInitialGuess() const { return du_inertia_; }
  math::fieldr<dim> const &GetSolution() const { return du_; }
  virtual void SolveTimestep();

  // SECT: During the timestep, optimizers may require such information
  real Energy(math::fieldr<dim> const &du) const;
  math::fieldr<dim> Gradient(math::fieldr<dim> const &du) const;
  math::vecxr GradientFlat(math::vecxr const &du_flat) const;
  math::sp_matxxr Hessian(math::fieldr<dim> const &du) const;

  real ResidualNorm(math::fieldr<dim> const &grad) const;
  real L2Residual(math::fieldr<dim> const& grad) const;
  real L1Residual(math::fieldr<dim> const& grad) const;
  real LinfResidual(math::fieldr<dim> const& grad) const;

protected:
  // Common data
  SPtr<TriMesh<dim>> mesh_;
  UPtr<ElasticityComputeBase<dim>> elasticity_;
  UPtr<TimestepSchemeBase<dim>> integration_scheme_;
  std::string elasticity_name_{"linear"}, device_{"cpu"};

  real youngs_{3e6}, poisson_ratio_{0.33}, density_{1e3};

  // State variables
  math::fieldr<dim> u_, u_back_;                ///< Displacement
  math::fieldr<dim> velocity_, velocity_back_;  ///< Velocity
  math::fieldr<dim> ext_accel_;                 ///< External acceleration

  // Solve Results.
  math::fieldr<dim> du_inertia_, du_;  ///< Stores the solution of displacement of displacement.

  // runtime parameters.
  math::vec2r u_lame_;
  math::sp_matxxr mass_matrix_;
  math::sp_matxxr mass_matrix_original_;
  real dt_{1e-3};  ///< The time step, should be formulated here because many initializers use dt
                   ///< such as Quasi Newton proposed in Liu17.
  real abs_tol_grad_; // Dynamically set according to rel_tol_grad.

  // Convergency Parameters:
  real rel_tol_grad_{0.02};
  real tol_var_{1e-12};
  TimestepConvergeNormKind converge_kind_{TimestepConvergeNormKind::kLinf};
  idx max_iter_{1000};
  bool verbose_{false};

private:
  bool has_time_step_begin_;

  template <template <idx> class ElasticModelTemplate,
            template <idx, template <idx> class> class Compute = ElasticityCompute_CPU>
  void SetupElasticity() {
    elasticity_ = std::make_unique<Compute<dim, ElasticModelTemplate>>(mesh_);
  }
};

}  // namespace ax::fem
