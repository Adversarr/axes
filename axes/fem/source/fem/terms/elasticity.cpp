#include "ax/fem/terms/elasticity.hpp"

#include <set>

#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/create_buffer.hpp"
#include "ax/fem/utils/gather_builder.hpp"
#include "ax/math/buffer_blas.hpp"
#include "ax/utils/cuda_helper.hpp"
#include "details/elasticity_impl.hpp"

namespace ax::fem {

using size3 = std::tuple<size_t, size_t, size_t>;
using size2 = std::pair<size_t, size_t>;

static std::vector<size3> determine_fillin(std::shared_ptr<Mesh> mesh) {
  std::vector<size3> fillin;
  auto elem_host = create_buffer<size_t>(BufferDevice::Host, mesh->GetElements()->Shape());
  copy(elem_host->View(), mesh->GetElements()->ConstView());
  auto [v, e] = make_view(mesh->GetVertices(), elem_host);

  auto [dim, nV, _1] = *v.Shape();
  auto [nVPE, nE, _2] = *e.Shape();
  fillin.reserve(nE * nVPE * nVPE);
  size_t cnt = 0;
  for (size_t elem = 0; elem < nE; ++elem) {
    for (size_t j = 0; j < nVPE; ++j) {
      for (size_t i = 0; i < nVPE; ++i) {
        fillin.emplace_back(e(i, elem), e(j, elem), cnt++);
      }
    }
  }

  std::sort(fillin.begin(), fillin.end(), [](const auto& a, const auto& b) {
    auto [row_a, col_a, unused_1] = a;
    auto [row_b, col_b, unused_2] = b;
    return row_a < row_b || (row_a == row_b && col_a < col_b);
  });

  return fillin;
}

ElasticityTerm::ElasticityTerm(std::shared_ptr<State> state, std::shared_ptr<Mesh> mesh)
    : TermBase(state, mesh) {
  auto device = state_->Device();

  // Initialize the gradient.
  gradient_ = state->GetVariables()->Clone();
  gradient_->SetBytes(0);

  // Initialize the Hessian.
  auto [bs, nv, _1] = *gradient_->Shape();
  hessian_ = math::RealBlockMatrix(nv, nv, bs, device);

  // Initialize the constraints.
  // For typical elasticity (not bending...), just the element
  constraints_ = mesh->GetElements()->Clone();

  // Initialize the rest position.
  rest_ = mesh->GetVertices()->Clone();

  // Initialize the rest volume.
  size_t n_vert = mesh->GetNumVertices();
  size_t n_elem = mesh->GetNumElements();
  size_t n_dof = mesh->GetNumDOFPerVertex();
  size_t n_vert_per_elem = mesh->GetNumVerticesPerElement();
  rest_volume_ = create_buffer<Real>(device, {n_elem});
  dminv_ = create_buffer<Real>(device, {n_dof, n_dof, n_elem});
  pfpx_ = create_buffer<Real>(device, {n_dof * n_dof, n_dof * n_vert_per_elem, n_elem});
  auto [r, rv, dm, p] = make_view(rest_, rest_volume_, dminv_, pfpx_);

  if (device == BufferDevice::Device) {
    AX_CUDA_CALL(compute_static_data_gpu(*mesh, r, rv, dm, p));
  } else {
    compute_static_data_cpu(*mesh, r, rv, dm, p);
  }

  // compute_
  size_t n_cubature = mesh->GetNumElements();
  compute_ = ElasticityBatchedCompute(n_cubature, n_dof, device);

  // elem_grad_
  elem_grad_ = create_buffer<Real>(device, {n_elem});

  // gather_gradient_
  elem_grad_ = create_buffer<Real>(device, {n_dof, n_vert_per_elem, n_elem});
  std::vector<Real> rest_volume(n_elem);
  copy(view_from_buffer(rest_volume), rv);
  size_t n_gather_grad_in = n_elem * n_vert_per_elem;
  gather_gradient_ = math::GatherAddOp(n_gather_grad_in, n_vert, n_gather_grad_in, device);

  auto elem_host = create_buffer<size_t>(BufferDevice::Host, mesh->GetElements()->Shape());
  copy(elem_host->View(), mesh->GetElements()->ConstView());

  {  // Setup gather gradient
    std::vector<size2> fillins;
    auto e = elem_host->View();
    for (size_t elem = 0; elem < n_elem; ++elem) {
      for (size_t i = 0; i < n_vert_per_elem; ++i) {
        auto v = e(i, elem);
        fillins.push_back(size2{v, elem * n_vert_per_elem + i});
      }
    }
    std::sort(fillins.begin(), fillins.end(), [](const size2& a, const size2& b) {
      return a.first < b.first || (a.first == b.first && a.second < b.second);
    });
    std::vector<size_t> row_entries(n_vert + 1, 0);
    std::vector<size_t> col_indices(fillins.size());
    std::vector<Real> weights(fillins.size(), 0);
    for (size_t i = 0; i < fillins.size(); ++i) {
      row_entries[fillins[i].first + 1]++;
      col_indices[i] = fillins[i].second;
      weights[i] = rest_volume[fillins[i].second / n_vert_per_elem];
    }
    // partial sum
    for (size_t i = 1; i < row_entries.size(); ++i) {
      row_entries[i] += row_entries[i - 1];
    }

    gather_gradient_.SetData(view_from_buffer(weights), view_from_buffer(row_entries),
                             view_from_buffer(col_indices));
  }

  // gather_hessian_
  auto fillins = determine_fillin(mesh);
  std::set<size2> fillin_set;
  for (const auto& f : fillins) {
    fillin_set.insert(size2{std::get<0>(f), std::get<1>(f)});
  }

  size_t n_fillin = fillin_set.size();
  size_t n_gather_hess_in = n_elem * n_vert_per_elem * n_vert_per_elem;
  AX_DCHECK(n_gather_hess_in == fillins.size(), "Mismatch in fillin size.");
  elem_hess_ = create_buffer<Real>(device, {n_dof, n_dof, n_gather_hess_in});
  gather_hessian_ = math::GatherAddOp(n_gather_hess_in, n_fillin, n_gather_hess_in, device);

  {  // Setup gather hessian
    std::vector<size_t> row_entries(n_fillin + 1, 0);
    std::vector<size_t> col_indices(n_gather_hess_in);
    std::vector<Real> weights(n_gather_hess_in, 0);

    size_t cnt = 0;
    for (size_t i = 0; i < n_gather_hess_in; ++i) {
      auto [this_row, this_col, from] = fillins[i];
      do {
        auto [row, col, from_i] = fillins[i];
        if (row != this_row || col != this_col) {
          --i;
          break;
        }

        ++row_entries[cnt + 1];
        col_indices[i] = from_i;
        weights[i] = rest_volume[from_i / (n_vert_per_elem * n_vert_per_elem)];

        ++i;
      } while (i < n_gather_hess_in);
      ++cnt;
    }
    AX_DCHECK(cnt == n_fillin, "Mismatch in cnt.");
    // partial sum
    for (size_t i = 1; i < row_entries.size(); ++i) {
      row_entries[i] += row_entries[i - 1];
    }
    AX_DCHECK(row_entries.back() == n_gather_hess_in, "Mismatch in row_entries.");

    gather_hessian_.SetData(view_from_buffer(weights), view_from_buffer(row_entries),
                            view_from_buffer(col_indices));
  }

  // Initialize the energy.
  energy_ = 0;
  gradient_->SetBytes(0);

  {  // hessian
    std::vector<int> row_entries(n_vert + 1, 0);
    std::vector<int> col_indices(n_fillin);
    std::vector<Real> values(n_fillin * n_dof * n_dof, 0);

    size_t cnt = 0;
    for (const auto& f : fillin_set) {
      auto [row, col] = f;
      ++row_entries[row + 1];
      col_indices[cnt] = static_cast<int>(col);
      ++cnt;
    }

    for (size_t i = 1; i < row_entries.size(); ++i) {
      row_entries[i] += row_entries[i - 1];
    }
    AX_DCHECK(cnt == n_fillin, "Mismatch in cnt.");
    AX_DCHECK(row_entries.back() == n_fillin, "Mismatch in row_entries.");

    hessian_.SetData(view_from_buffer(row_entries), view_from_buffer(col_indices),
                     view_from_buffer(values, {n_dof, n_dof, n_fillin}));
  }

  is_energy_up_to_date_ = false;
  is_gradient_up_to_date_ = false;
  is_hessian_up_to_date_ = false;
}

void ElasticityTerm::UpdateEnergy() {
  if (is_energy_up_to_date_) {
    return;
  }
  auto device = state_->Device();

  auto u = state_->GetVariables()->ConstView();
  auto f = compute_.DeformGrad()->View();
  auto dminv = dminv_->ConstView();
  if (device == BufferDevice::Device) {
    AX_CUDA_CALL(compute_deformation_gradient_gpu(*mesh_, dminv, u, f));
  } else {
    compute_deformation_gradient_cpu(*mesh_, dminv, u, f);
  }
  compute_.UpdateDeformationGradient();
  compute_.UpdateEnergyDensity();

  auto [e, r] = make_view(compute_.EnergyDensity(), rest_volume_);
  energy_ = math::buffer_blas::dot(e, r);
  is_energy_up_to_date_ = true;
}

void ElasticityTerm::UpdateGradient() {
  if (is_gradient_up_to_date_) {
    return;
  }
  auto device = state_->Device();

  auto u = state_->GetVariables()->ConstView();
  auto f = compute_.DeformGrad()->View();
  auto dminv = dminv_->ConstView();
  if (device == BufferDevice::Device) {
    AX_CUDA_CALL(compute_deformation_gradient_gpu(*mesh_, dminv, u, f));
  } else {
    compute_deformation_gradient_cpu(*mesh_, dminv, u, f);
  }
  compute_.UpdateDeformationGradient();
  compute_.UpdateGradient();
  compute_.UpdateEnergyDensity();

  auto [grad, r, elem_grad] = make_view(compute_.Pk1(), rest_volume_, elem_grad_);
  if (device == BufferDevice::Device) {
    AX_CUDA_CALL(compute_cubature_gradient_gpu(*mesh_, grad, dminv, elem_grad));
  } else {
    compute_cubature_gradient_cpu(*mesh_, grad, dminv, elem_grad);
  }
  size_t n_elem = mesh_->GetNumElements(), n_vert_per_elem = mesh_->GetNumVerticesPerElement();
  size_t n_dof = mesh_->GetNumDOFPerVertex();
  gather_gradient_.Apply(elem_grad.Reshaped({n_dof, n_vert_per_elem * n_elem}), gradient_->View(),
                         1, 0);
  auto e = compute_.EnergyDensity()->ConstView();
  energy_ = math::buffer_blas::dot(e, r);

  is_gradient_up_to_date_ = true;
  is_energy_up_to_date_ = true;
}

void ElasticityTerm::UpdateHessian() {
  if (is_hessian_up_to_date_) {
    return;
  }
  auto device = state_->Device();

  auto u = state_->GetVariables()->ConstView();
  auto f = compute_.DeformGrad()->View();
  auto dminv = dminv_->ConstView();
  if (device == BufferDevice::Device) {
    AX_CUDA_CALL(compute_deformation_gradient_gpu(*mesh_, dminv, u, f));
  } else {
    compute_deformation_gradient_cpu(*mesh_, dminv, u, f);
  }
  compute_.UpdateDeformationGradient();
  compute_.UpdateHessian();
  compute_.UpdateGradient();
  compute_.UpdateEnergyDensity();

  auto [hess, r, elem_hess] = make_view(compute_.LocalHessian(), rest_volume_, elem_hess_);
  auto [grad, elem_grad] = make_view(compute_.Pk1(), elem_grad_);
  auto e = compute_.EnergyDensity()->ConstView();
  // Integrates
  if (device == BufferDevice::Device) {
    AX_CUDA_CALL(compute_cubature_hessian_gpu(*mesh_, hess, pfpx_->View(), elem_hess));
    AX_CUDA_CALL(compute_cubature_gradient_gpu(*mesh_, grad, dminv, elem_grad));
  } else {
    compute_cubature_hessian_cpu(*mesh_, hess, pfpx_->View(), elem_hess);
    compute_cubature_gradient_cpu(*mesh_, grad, dminv, elem_grad);
  }
  // Gather from cubatures.
  size_t n_elem = mesh_->GetNumElements(), n_vert_per_elem = mesh_->GetNumVerticesPerElement();
  size_t n_dof = mesh_->GetNumDOFPerVertex();
  gather_hessian_.Apply(elem_hess, hessian_.Values()->View(), 1, 0);
  gather_gradient_.Apply(elem_grad.Reshaped({n_dof, n_vert_per_elem * n_elem}), gradient_->View(),
                         1, 0);
  energy_ = math::buffer_blas::dot(e, r);

  is_hessian_up_to_date_ = true;
  is_gradient_up_to_date_ = true;
  is_energy_up_to_date_ = true;
}

}  // namespace ax::fem