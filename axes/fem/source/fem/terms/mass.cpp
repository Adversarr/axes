#include "ax/fem/terms/mass.hpp"

#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/create_buffer.hpp"
#include "ax/core/buffer/eigen_support.hpp"
#include "ax/core/buffer/for_each.hpp"
#include "ax/fem/elements/p1.hpp"
#include "ax/math/buffer_blas.hpp"
#include "ax/math/sparse_matrix/csr.hpp"
#include "details/mass_impl.hpp"

#include <gsl/pointers>

namespace ax::fem {

MassTerm::MassTerm(shared_not_null<State> state, shared_not_null<Mesh> mesh)
    : TermBase(state, mesh) {
  auto device = state_->GetVariables()->Device();

  // Initialize the gradient.
  gradient_ = state->GetVariables()->Clone();
  gradient_->SetBytes(0);

  // Initialize the Hessian.
  auto [bs, nv, _] = *gradient_->Shape();
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

  SetDensity(1.0); // Default density is 1.0.
}

void MassTerm::MarkDirty() {
  is_diff_up_to_date_ = false;
  is_energy_up_to_date_ = false;
  is_gradient_up_to_date_ = false;
  // is_hessian_up_to_date_ = false; // No need to update the Hessian.
}

void MassTerm::UpdateEnergy() {
  if (is_energy_up_to_date_) {
    return;
  }

  auto [bs, nv, _] = *gradient_->Shape();
  // compute diff.
  auto [rhs, diff, u, grad] = make_view(rhs_, diff_, state_->GetVariables(), gradient_);
  // diff <- u - rhs.
  math::buffer_blas::copy(diff, u);          // diff = u
  math::buffer_blas::axpy(-1.0, rhs, diff);  // diff = u - rhs

  // grad <- M(u-rhs)
  hessian_.Multiply(diff, grad, 1.0, 0.0);

  // compute energy.
  energy_ = 0.5 * math::buffer_blas::dot(diff, grad);
  is_diff_up_to_date_ = true;
  is_energy_up_to_date_ = true;
  is_gradient_up_to_date_ = true;
  is_hessian_up_to_date_ = true;
}

void MassTerm::UpdateGradient() {
  if (!is_gradient_up_to_date_) {
    UpdateEnergy();
  }
}

void MassTerm::UpdateHessian() {
  // Nothing to do.
}

void MassTerm::SetDensity(ConstRealBufferView uniform_density) {
  AX_THROW_IF(
      !is_1d(uniform_density.Shape()) || uniform_density.Shape().X() != mesh_->GetNumElements(),
      "MassTerm: Density must be a 1D buffer with the same number of elements as the mesh.");
  // Create the mass matrix.
  hessian_ = details::compute_mass_matrix_host(*mesh_, uniform_density,
                                               state_->GetVariables()->Shape().X());
  hessian_.MarkAsSymmetric();
}

void MassTerm::SetDensity(Real uniform_density) {
  std::vector<Real> density(mesh_->GetNumElements(), uniform_density);
  SetDensity(view_from_buffer(density));
}

}  // namespace ax::fem
