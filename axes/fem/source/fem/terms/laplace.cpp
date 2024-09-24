#include "ax/fem/terms/laplace.hpp"

#include "ax/core/gsl.hpp"
#include "ax/math/buffer_blas.hpp"
#include "details/laplace_impl.hpp"

namespace ax::fem {

LaplaceTerm::LaplaceTerm(shared_not_null<State> state, shared_not_null<Mesh> mesh)
    : TermBase(state, mesh) {
  auto device = state_->GetVariables()->Device();

  // Initialize the gradient.
  gradient_ = state->GetVariables()->Clone();
  gradient_->SetBytes(0);

  // Initialize the Hessian.
  auto [bs, nv, _1] = *gradient_->Shape();
  hessian_ = math::RealBlockMatrix(nv, nv, bs, device);

  // Initialize the constraints.
  // For mass matrix, just the element
  constraints_ = mesh->GetElements()->Clone();

  // Initialize the rhs
  rhs_ = state->GetVariables()->Clone();
  rhs_->SetBytes(0);

  // Initialize the temp buffer, used for store the immediate result
  diff_ = state->GetVariables()->Clone();
  diff_->SetBytes(0);

  // Default density is 1.0.
  SetDiffusivity(1.0);
}

void LaplaceTerm::MarkDirty() {
  is_diff_up_to_date_ = false;
  is_energy_up_to_date_ = false;
  is_gradient_up_to_date_ = false;
  // is_hessian_up_to_date_ = false; // No need to update the Hessian.
}

void LaplaceTerm::UpdateEnergy() {
  if (is_energy_up_to_date_) {
    return;
  }

  auto [bs, nv, _1] = *gradient_->Shape();
  // compute diff.
  auto [rhs, diff, u, grad] = make_view(rhs_, diff_, state_->GetVariables(), gradient_);
  // compute: grad <- L u - rhs
  hessian_.Multiply(u, diff, 1.0, 0.0);      // diff <- L u
  math::buffer_blas::copy(grad, diff);       // grad <- diff = L u
  math::buffer_blas::axpy(-1.0, rhs, grad);  // grad = L u - rhs

  // compute energy: 1/2 u.T L u - u.T rhs.
  energy_ = 0.5 * math::buffer_blas::dot(u, diff) - math::buffer_blas::dot(u, rhs);
  is_diff_up_to_date_ = true;
  is_energy_up_to_date_ = true;
  is_gradient_up_to_date_ = true;
  is_hessian_up_to_date_ = true;
}

void LaplaceTerm::UpdateGradient() {
  if (!is_gradient_up_to_date_) {
    UpdateEnergy();
  }
}

void LaplaceTerm::UpdateHessian() {
  // Nothing to do.
}

void LaplaceTerm::SetDiffusivity(ConstRealBufferView uniform_diffusivity) {
  AX_THROW_IF(!is_1d(uniform_diffusivity.Shape())
                  || uniform_diffusivity.Shape().X() != mesh_->GetNumElements(),
              "LaplaceTerm: Diffusivity must be a 1D buffer with the same number of elements as "
              "the mesh. expect {} got {}",
              mesh_->GetNumElements(), uniform_diffusivity.Shape().X());
  auto hess = details::compute_laplace_matrix_host(*mesh_, uniform_diffusivity,
                                                   state_->GetVariables()->Shape().X());
  hessian_.SetData(hess.RowPtrs()->ConstView(), hess.ColIndices()->ConstView(),
                   hess.Values()->ConstView());
  hessian_.MarkAsSymmetric();
  hessian_.Finish();
}

void LaplaceTerm::SetDiffusivity(Real uniform_diffusivity) {
  std::vector<Real> diffusivity(mesh_->GetNumElements(), uniform_diffusivity);
  SetDiffusivity(view_from_buffer(diffusivity));
}

}  // namespace ax::fem