#include "ax/fem/terms/mass.hpp"

#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/create_buffer.hpp"
#include "ax/core/buffer/eigen_support.hpp"
#include "ax/core/buffer/for_each.hpp"
#include "ax/fem/elements/p1.hpp"
#include "ax/math/buffer_blas.hpp"
#include "ax/math/sparse_matrix/csr.hpp"

namespace ax::fem {

MassTerm::MassTerm(std::shared_ptr<State> state, std::shared_ptr<Mesh> mesh)
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
  hessian_.RightMultiplyTo(diff, grad, 1.0, 0.0);

  // compute energy.
  energy_ = 0.5 * math::buffer_blas::dot(diff, grad);
  is_diff_up_to_date_ = true;
  is_energy_up_to_date_ = true;
  is_gradient_up_to_date_ = true;
}

void MassTerm::UpdateGradient() {
  if (!is_gradient_up_to_date_) {
    UpdateEnergy();
  }
}

void MassTerm::UpdateHessian() {
  // Nothing to do.
}

math::RealSparseCOO compute_fill_in_2d(ConstRealBufferView position, ConstSizeBufferView elements,
                                       ConstRealBufferView density) {
  math::RealSparseCOO coo;
  auto elem = view_as_matrix_1d<const size_t, 3>(elements);
  auto pos = view_as_matrix_1d<const Real, 2>(position);
  size_t n_elem = elements.Shape().Y();
  for (size_t i = 0; i < n_elem; ++i) {
    auto v0 = pos(elem(i)(0));
    auto v1 = pos(elem(i)(1));
    auto v2 = pos(elem(i)(2));
    elements::P1Element2D p1({v0, v1, v2});
    for (Index k = 0; k < 3; ++k) {
      for (Index l = 0; l < 3; ++l) {
        auto vk = elem(i)(k), vl = elem(i)(l);
        Real val = p1.Integrate_F_F(k, l) * density(i);
        if (val != 0) {
          coo.push_back(math::RealSparseEntry(static_cast<Index>(vk), static_cast<Index>(vl), val));
        }
      }
    }
  }
  return coo;
}

void MassTerm::SetDensity(ConstRealBufferView uniform_density) {
  auto [dim, nv, _] = *gradient_->Shape();
  auto device = state_->GetVariables()->Device();

  if (dim != 2 && dim != 3) {
    throw make_runtime_error("MassTerm: Only support 2D and 3D. got {}", dim);
  }

  BufferPtr<Real> host_position = create_buffer<Real>(BufferDevice::Host, {nv, dim});
  copy(host_position->View(), mesh_->GetVertices()->ConstView());
  math::RealSparseCOO coo;

  if (dim == 2) {
    coo = compute_fill_in_2d(host_position->ConstView(), mesh_->GetElements()->ConstView(),
                             uniform_density);
  } else {
    // TODO.
  }
  math::RealCSRMatrix csr(nv, nv, BufferDevice::Host);
  csr.SetFromTriplets(coo);

  auto v = csr.GetValues();
  BufferPtr<Real> bsr_val = create_buffer<Real>(device, {dim, dim, v->Shape().X()});
  // TODO. not implemented.
}

}  // namespace ax::fem
