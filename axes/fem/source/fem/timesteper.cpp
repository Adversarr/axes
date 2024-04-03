#include "ax/math/linsys/sparse/ConjugateGradient.hpp"
#include "ax/math/linsys/sparse/LDLT.hpp"
#include "ax/fem/elasticity/linear.hpp"
#include "ax/fem/elasticity/neohookean_bw.hpp"
#include "ax/fem/timestepper.hpp"
#include "ax/fem/mass_matrix.hpp"
namespace ax::pde::fem {

template<idx dim>
TimeStepperBase<dim>::TimeStepperBase(UPtr<MeshBase<dim>> mesh)
    : mesh_(std::move(mesh))
{
}

template <idx dim>
Status TimeStepperBase<dim>::Init(utils::Opt const&) {
  idx n_vert = mesh_->GetNumVertices();
  deform_ = std::make_unique<Deformation<dim>>(*mesh_, mesh_->GetVertices());
  elasticity_ = std::make_unique<ElasticityCompute<dim, elasticity::Linear>>(*deform_);
  velocity_.resize(dim, n_vert);
  velocity_.setZero();
  auto mmc = MassMatrixCompute<dim>(*mesh_);
  mass_matrix_ = math::make_sparse_matrix(dim * n_vert, dim * n_vert, mmc(1.0e2));
  // setup the external force.
  math::vecxr flat_ext_force = math::vecxr::Zero(dim * n_vert);
  for (idx i = 0; i < n_vert; ++i) {
    flat_ext_force(dim * i + 1) = -9.8;
  }
  auto ext_force_flat = mass_matrix_ * flat_ext_force;
  ext_force_ = ext_force_flat.reshaped(dim, n_vert);
  AX_RETURN_OK();
}


template<idx dim>
Status TimeStepperBase<dim>::Step(real dt) {
  auto lame = pde::elasticity::compute_lame(1e5, 0.40);
  idx n_vert = mesh_->GetNumVertices();
  // Compute the external force. now we use -9.8 in y-direction.
  // Compute the internal force.
  elasticity_->UpdateDeformationGradient(DeformationGradientUpdate::kStress);
  auto stress_on_elements = elasticity_->Stress(lame);
  math::vecxr force_on_vertices = -(deform_->StressToVertices(stress_on_elements)).reshaped();
  auto hessian_on_elements = elasticity_->Hessian(lame);
  math::sp_coeff_list hessian_coo = deform_->HessianToVertices(hessian_on_elements);
  auto K = math::make_sparse_matrix(dim * n_vert, dim * n_vert, hessian_coo);

  // Solve the acceleration.
  const auto & M = mass_matrix_;
  auto V = velocity_.reshaped();
  math::SparseSolver_ConjugateGradient solver;
  math::LinsysProblem_Sparse prob_mass;
  prob_mass.A_ = M + K * dt * dt;
  prob_mass.b_ = M * dt * V + dt * dt * (force_on_vertices + ext_force_.reshaped());
  mesh_->FilterVector(prob_mass.b_, true);
  mesh_->FilterMatrix(prob_mass.A_);
  auto dx_flat = solver.SolveProblem(prob_mass);
  if (! dx_flat.ok()) {
    return dx_flat.status();
  }
  math::fieldr<dim> x_new = dx_flat->solution_.reshaped(dim, n_vert) + mesh_->GetVertices();

  // Compute the velocity.
  velocity_ = (x_new - mesh_->GetVertices()) / dt;

  // Update the position of mesh.
  AX_RETURN_NOTOK(mesh_->SetVertices(x_new));
  AX_RETURN_OK();
}

template class TimeStepperBase<2>;
template class TimeStepperBase<3>;

}
