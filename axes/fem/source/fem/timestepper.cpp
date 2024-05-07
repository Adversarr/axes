#include "ax/fem/timestepper.hpp"

#include "ax/fem/elasticity/linear.hpp"
#include "ax/fem/elasticity/neohookean_bw.hpp"
#include "ax/fem/mass_matrix.hpp"
#include "ax/math/linsys/sparse/ConjugateGradient.hpp"
#include "ax/math/linsys/sparse/LDLT.hpp"
namespace ax::fem {

template <idx dim> TimeStepperBase<dim>::TimeStepperBase(SPtr<TriMesh<dim>> mesh)
    : mesh_(std::move(mesh)) {
  integration_scheme_ = TimestepSchemeBase<dim>::Create(TimestepSchemeKind::kBackwardEuler);
}

template <idx dim> Status TimeStepperBase<dim>::Init(utils::Opt const&) {
  idx n_vert = mesh_->GetNumVertices();
  SetupElasticity<elasticity::Linear>();

  u_.setZero(dim, n_vert);
  u_back_ = u_;
  du_ = u_;
  du_inertia_ = u_;
  velocity_.setZero(dim, n_vert);
  velocity_back_ = velocity_;
  ext_accel_.setZero(dim, n_vert);

  if (mass_matrix_.size() == 0) {
    AX_LOG(WARNING) << "Density not set. Default to 1.0e3.";
    SetDensity(1.0e3);
  }
  AX_RETURN_OK();
}

template <idx dim> void TimeStepperBase<dim>::SetDensity(real density) {
  auto mmc = MassMatrixCompute<dim>(*mesh_);
  mass_matrix_original_ = mmc(density);
  mass_matrix_ = math::kronecker_identity<dim>(mass_matrix_original_);
}

template <idx dim> void TimeStepperBase<dim>::SetDensity(math::field1r const& density) {
  auto mmc = MassMatrixCompute<dim>(*mesh_);
  mass_matrix_original_ = mmc(density);
  mass_matrix_ = math::kronecker_identity<dim>(mass_matrix_original_);
}

template <idx dim> Status TimeStepperBase<dim>::Step(real dt) {
  auto lame = u_lame_;
  idx n_vert = mesh_->GetNumVertices();
  // Compute the external force. now we use -9.8 in y-direction.
  // Compute the internal force.
  auto stress_on_elements = elasticity_->Stress(lame);
  math::vecxr neg_force = elasticity_->GatherStress(stress_on_elements).reshaped();
  auto hessian_on_elements = elasticity_->Hessian(lame);
  math::sp_matxxr K = elasticity_->GatherHessian(hessian_on_elements);

  // Solve the new (velocity * dt) = (x' - x)
  const auto& M = mass_matrix_;
  math::vecxr V = velocity_.reshaped();
  math::vecxr X = mesh_->GetVertices().reshaped();
  math::vecxr Y = dt * V + dt * dt * ext_accel_.reshaped();
  mesh_->FilterVector(Y, true);
  math::SparseSolver_ConjugateGradient solver;
  math::LinsysProblem_Sparse linsys;
  linsys.A_ = (M + K * dt * dt);
  linsys.b_ = M * Y - neg_force * dt * dt;
  mesh_->FilterVector(linsys.b_, true);
  mesh_->FilterMatrixFull(linsys.A_);

  auto dx_flat = solver.SolveProblem(linsys);
  if (!dx_flat.ok()) {
    return dx_flat.status();
  }
  math::fieldr<dim> x_new = (X + dx_flat->solution_).reshaped(dim, n_vert);
  // Compute the velocity.
  velocity_ = (x_new - mesh_->GetVertices()) / dt;
  // std::cout << "|Velocity|: " << math::norm(velocity_) << std::endl;

  // Update the position of mesh.
  AX_RETURN_NOTOK(mesh_->SetVertices(x_new));
  AX_RETURN_OK();
}

template <idx dim> Status TimeStepperBase<dim>::Precompute() { AX_RETURN_OK(); }

template <idx dim>
math::sp_matxxr TimeStepperBase<dim>::GetStiffnessMatrix(math::fieldr<dim> const& x,
                                                         bool project) const {
  auto lame = u_lame_;
  elasticity_->Update(x, ElasticityUpdateLevel::kHessian);
  elasticity_->UpdateHessian(project);
  elasticity_->GatherHessianToVertices();
  return elasticity_->GetHessianOnVertices();
}

template <idx dim>
math::fieldr<dim> TimeStepperBase<dim>::GetElasticForce(math::fieldr<dim> const& x) const {
  auto lame = u_lame_;
  elasticity_->Update(x, ElasticityUpdateLevel::kStress);
  elasticity_->UpdateStress();
  elasticity_->GatherStressToVertices();
  return elasticity_->GetStressOnVertices();
}

template <idx dim> math::fieldr<dim> TimeStepperBase<dim>::GetInertiaPosition(real dt) const {
  return mesh_->GetVertices() + dt * velocity_ + dt * dt * ext_accel_;
}

template <idx dim> void TimeStepperBase<dim>::BeginSimulation(real dt) {
  has_time_step_begin_ = false;
  return;
}

template <idx dim> void TimeStepperBase<dim>::BeginTimestep(real dt) {
  if (has_time_step_begin_) {
    throw LogicError("Timestep has already begun. Call EndTimestep before starting a new one.");
  }
  integration_scheme_->SetDeltaT(dt);
  du_inertia_ = integration_scheme_->Precomputed(mass_matrix_original_, u_, u_back_, velocity_,
                                                 velocity_back_, ext_accel_);
  du_ = integration_scheme_->InitialGuess(u_, u_back_, velocity_, velocity_back_, ext_accel_);
  // TODO: Lame parameters should be set uniformly or per element.
  elasticity_->SetLame(u_lame_);

  // convergency test parameters
  idx n_vert = mesh_->GetNumVertices();
  math::fieldr<dim> body_force = (math::fieldr<dim>::Ones(dim, n_vert) * mass_matrix_original_);
  abs_tol_grad_ = dt * dt * ResidualNorm(body_force) + math::epsilon<real>;

  has_time_step_begin_ = true;
  return;
}

template <idx dim> void TimeStepperBase<dim>::EndTimestep() {
  velocity_back_ = integration_scheme_->NewVelocity(u_, u_back_, velocity_, velocity_back_, du_);
  std::swap(u_back_, u_);
  std::swap(velocity_back_, velocity_);
  u_.noalias() = du_ + u_back_;
  has_time_step_begin_ = false;
}

template <idx dim> math::fieldr<dim> const& TimeStepperBase<dim>::SolveTimestep() {
  // Do nothing, do not throw anything, for testing.
  AX_LOG(ERROR) << "SolveTimestep is not implemented!";
  du_.setZero();
  return du_;
}

template <idx dim> real TimeStepperBase<dim>::Energy(math::fieldr<dim> const& du) const {
  math::fieldr<dim> x_new = du + u_ + mesh_->GetVertices();
  elasticity_->Update(x_new, ElasticityUpdateLevel::kEnergy);
  elasticity_->UpdateEnergy();
  real stiffness = elasticity_->GetEnergyOnElements().sum();
  math::fieldr<dim> duu = du - du_inertia_;
  math::fieldr<dim> Mdx = duu * mass_matrix_original_;
  real inertia = (duu.array() * Mdx.array()).sum();
  return integration_scheme_->ComposeEnergy(inertia, stiffness);
}

template <idx dim>
math::fieldr<dim> TimeStepperBase<dim>::Gradient(math::fieldr<dim> const& du) const {
  math::fieldr<dim> u_cur = du + u_;
  math::fieldr<dim> x_new = u_cur + mesh_->GetVertices();

  elasticity_->Update(x_new, ElasticityUpdateLevel::kStress);
  elasticity_->UpdateStress();
  elasticity_->GatherStressToVertices();
  math::fieldr<dim> neg_force = elasticity_->GetStressOnVertices();
  return integration_scheme_->ComposeGradient(mass_matrix_original_, u_ + du, neg_force,
                                              du_inertia_);
}

template <idx dim> math::vecxr TimeStepperBase<dim>::GradientFlat(math::vecxr const& du) const {
  idx n_vert = mesh_->GetNumVertices();
  return Gradient(du.reshaped(dim, n_vert)).reshaped();
}

template <idx dim>
math::sp_matxxr TimeStepperBase<dim>::Hessian(math::fieldr<dim> const& du) const {
  auto x_cur = u_ + mesh_->GetVertices();
  math::fieldr<dim> x_new = du + x_cur;
  elasticity_->Update(x_new, ElasticityUpdateLevel::kHessian);
  elasticity_->UpdateHessian(true);
  elasticity_->GatherHessianToVertices();
  auto stiffness = elasticity_->GetHessianOnVertices();
  return integration_scheme_->ComposeHessian(mass_matrix_original_, stiffness);
}

template <idx dim> optim::OptProblem TimeStepperBase<dim>::AssembleProblem() const {
  optim::OptProblem problem;
  problem
      .SetEnergy([this](math::vecxr const& du) -> real {
        return Energy(du.reshaped(dim, mesh_->GetNumVertices()));
      })
      .SetGrad([this](math::vecxr const& du) -> math::vecxr { return GradientFlat(du); })
      .SetSparseHessian([this](math::vecxr const& du) -> math::sp_matxxr {
        return Hessian(du.reshaped(dim, mesh_->GetNumVertices()));
      })
      .SetConvergeGrad([this](const math::vecxr& du, const math::vecxr& grad) -> real {
        return ResidualNorm(grad) / abs_tol_grad_;
      })
      .SetConvergeVar([this](const math::vecxr& du_back, const math::vecxr& du) -> real {
        return (du_back - du).cwiseAbs().maxCoeff();
      });

  return problem;
}

template <idx dim> real TimeStepperBase<dim>::ResidualNorm(math::fieldr<dim> const& grad) const {
  switch (converge_kind_) {
    case TimestepConvergeNormKind::kL1:
      return L1Residual(grad);
    case TimestepConvergeNormKind::kLinf:
      return LinfResidual(grad);
    case TimestepConvergeNormKind::kL2:
    default:
      return L2Residual(grad);
  }
}

template <idx dim> real TimeStepperBase<dim>::L2Residual(math::fieldr<dim> const& grad) const {
  return math::norm(grad, math::l2_t{});
}

template <idx dim> real TimeStepperBase<dim>::L1Residual(math::fieldr<dim> const& grad) const {
  return math::norm(grad, math::l1_t{});
}

template <idx dim> real TimeStepperBase<dim>::LinfResidual(math::fieldr<dim> const& grad) const {
  return math::norm(grad, math::linf_t{});
}

template class TimeStepperBase<2>;
template class TimeStepperBase<3>;

}  // namespace ax::fem
