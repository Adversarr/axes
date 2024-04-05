#include "ax/math/linsys/sparse/ConjugateGradient.hpp"
#include "ax/math/linsys/sparse/LDLT.hpp"
#include "ax/fem/elasticity/linear.hpp"
#include "ax/fem/elasticity/neohookean_bw.hpp"
#include "ax/fem/timestepper.hpp"
#include "ax/fem/mass_matrix.hpp"
namespace ax::fem {

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

template<idx dim>
void TimeStepperBase<dim>::SetDensity(real density) {
  auto mmc = MassMatrixCompute<dim>(*mesh_);
  mass_matrix_
      = math::make_sparse_matrix(dim * mesh_->GetNumVertices(), dim * mesh_->GetNumVertices(),
                                 mmc(density));
}

template <idx dim>
void TimeStepperBase<dim>::SetDensity(math::field1r const& density, bool is_density_on_elements) {
  auto mmc = MassMatrixCompute<dim>(*mesh_);
  mass_matrix_
      = math::make_sparse_matrix(dim * mesh_->GetNumVertices(), dim * mesh_->GetNumVertices(),
                                 mmc(density, is_density_on_elements));
}


template<idx dim>
Status TimeStepperBase<dim>::Step(real dt) {
  auto lame = lame_;
  idx n_vert = mesh_->GetNumVertices();
  // Compute the external force. now we use -9.8 in y-direction.
  // Compute the internal force.
  elasticity_->UpdateDeformationGradient(DeformationGradientUpdate::kStress);
  auto stress_on_elements = elasticity_->Stress(lame);
  math::vecxr neg_force = deform_->StressToVertices(stress_on_elements).reshaped();
  auto hessian_on_elements = elasticity_->Hessian(lame);
  math::sp_coeff_list hessian_coo = deform_->HessianToVertices(hessian_on_elements);
  auto K = math::make_sparse_matrix(dim * n_vert, dim * n_vert, hessian_coo);

  // Solve the new (velocity * dt) = (x' - x)
  const auto & M = mass_matrix_;
  math::vecxr V = velocity_.reshaped();
  math::vecxr X = mesh_->GetVertices().reshaped();
  math::vecxr Y = dt * V + dt * dt * ext_accel_.reshaped();
  mesh_->FilterVector(Y, true);
  math::SparseSolver_ConjugateGradient solver;
  math::LinsysProblem_Sparse linsys;
  linsys.A_ = M + K * dt * dt;
  linsys.b_ = M * Y - neg_force * dt * dt;
  mesh_->FilterVector(linsys.b_, true);
  mesh_->FilterMatrix(linsys.A_);

  auto dx_flat = solver.SolveProblem(linsys);
  if (! dx_flat.ok()) {
    return dx_flat.status();
  }
  math::fieldr<dim> x_new = (X + dx_flat->solution_).reshaped(dim, n_vert);
  // Compute the velocity.
  velocity_ = (x_new - mesh_->GetVertices()) / dt;
  std::cout << "|Velocity|: " << math::norm(velocity_) << std::endl;

  // Update the position of mesh.
  AX_RETURN_NOTOK(mesh_->SetVertices(x_new));
  AX_RETURN_OK();
}

template class TimeStepperBase<2>;
template class TimeStepperBase<3>;

}
