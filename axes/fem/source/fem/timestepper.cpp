#include "ax/fem/timestepper.hpp"

#include "ax/fem/elasticity/linear.hpp"
#include "ax/fem/elasticity/neohookean_bw.hpp"
#include "ax/fem/mass_matrix.hpp"
#include "ax/math/linsys/sparse/ConjugateGradient.hpp"
#include "ax/math/linsys/sparse/LDLT.hpp"
namespace ax::fem {

template <idx dim> TimeStepperBase<dim>::TimeStepperBase(SPtr<TriMesh<dim>> mesh)
    : mesh_(std::move(mesh)) {}

template <idx dim> Status TimeStepperBase<dim>::Init(utils::Opt const&) {
  idx n_vert = mesh_->GetNumVertices();
  SetupElasticity<elasticity::Linear>();

  velocity_.setZero(dim, n_vert);
  // setup the external force. (Default is Gravity only.)
  ext_accel_.setZero(dim, n_vert);
  for (idx i = 0; i < n_vert; ++i) {
    ext_accel_(1, i) = -9.8;
  }

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

template<idx dim> math::sp_matxxr TimeStepperBase<dim>::GetStiffnessMatrix(math::fieldr<dim> const& x, bool project) const {
  auto lame = u_lame_;
  elasticity_->Update(x, ElasticityUpdateLevel::kHessian);
  elasticity_->UpdateHessian(project);
  elasticity_->GatherHessianToVertices();
  return elasticity_->GetHessianOnVertices();
}

template<idx dim> math::fieldr<dim> TimeStepperBase<dim>::GetElasticForce(math::fieldr<dim> const& x) const {
  auto lame = u_lame_;
  elasticity_->Update(x, ElasticityUpdateLevel::kStress);
  elasticity_->UpdateStress();
  elasticity_->GatherStressToVertices();
  return elasticity_->GetStressOnVertices();
}

template <idx dim> math::fieldr<dim> TimeStepperBase<dim>::GetInertiaPosition(real dt) const {
  return mesh_->GetVertices() + dt * velocity_ + dt * dt * ext_accel_;
}

template class TimeStepperBase<2>;
template class TimeStepperBase<3>;

}  // namespace ax::fem
