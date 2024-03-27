#include "axes/math/linsys/sparse/ConjugateGradient.hpp"
#include "axes/math/linsys/sparse/LDLT.hpp"
#include "axes/pde/elasticity/neohookean_bw.hpp"
#include "axes/pde/fem/timestepper.hpp"
#include "axes/pde/fem/mass_matrix.hpp"
namespace ax::pde::fem {

template<idx dim>
TimeStepperBase<dim>::TimeStepperBase(UPtr<MeshBase<dim>> mesh)
    : mesh_(std::move(mesh))
{
  deform_ = std::make_unique<Deformation<dim>>(*mesh_, mesh_->GetVertices());
  // TODO: This should be a parameter.
  elasticity_ = std::make_unique<ElasticityCompute<dim, elasticity::NeoHookeanBW>>(*deform_);
}

template <idx dim>
Status TimeStepperBase<dim>::Init(utils::Opt const&) {
  idx n_vert = mesh_->GetNumVertices();
  velocity_.resize(3, n_vert);
  velocity_.setZero();
  AX_RETURN_OK();
}

template<idx dim>
Status TimeStepperBase<dim>::Step(real dt) {
  // We use the most simple way to solve, i.e. Predictor-Corrector.
  auto lame = pde::elasticity::compute_lame(1e4, 0.45);
  idx n_vert = mesh_->GetNumVertices();
  math::sp_matxxr massmat;
  {
    auto mmc = MassMatrixCompute<dim>(*mesh_);
    massmat = math::make_sparse_matrix(3 * n_vert, 3 * n_vert, mmc(1.0e3));
  }
  // Compute the external force. now we use -9.8 in y-direction.
  // Compute the internal force.
  elasticity_->UpdateDeformationGradient(DeformationGradientUpdate::kStress);
  auto stress_on_elements = elasticity_->Stress(lame);
  math::vecxr force_on_vertices = -(deform_->StressToVertices(stress_on_elements)).reshaped();
  // Solve the acceleration.
  math::SparseSolver_ConjugateGradient solver;
  math::LinsysProblem_Sparse prob_mass;
  prob_mass.A_ = massmat;
  prob_mass.b_ = force_on_vertices;
  auto acc_internal = solver.SolveProblem(prob_mass);
  if (! acc_internal.ok()) {
    return acc_internal.status();
  }
  math::fieldr<dim> accel_int = acc_internal->solution_.reshaped(3, n_vert);
  // Compute the velocity.
  velocity_ += dt * accel_int;
  velocity_.row(2).array() -= 9.8 * dt;

  // Update the position of mesh.
  auto cur = mesh_->GetVertices();
  cur += dt * velocity_;
  AX_RETURN_NOTOK(mesh_->SetVertices(cur));
  AX_RETURN_OK();
}

template class TimeStepperBase<2>;
template class TimeStepperBase<3>;

}