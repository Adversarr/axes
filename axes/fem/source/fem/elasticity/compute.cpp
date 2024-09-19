#include "ax/fem/elasticity/compute.hpp"

#include <gsl/gsl>

#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/create_buffer.hpp"
#include "ax/utils/cuda_helper.hpp"
#include "elast_impl.hpp"

namespace ax::fem {

std::map<ElasticityKind, bool> requires_svd = {
  {ElasticityKind::Linear, false},
  {ElasticityKind::IsotropicARAP, true},
  {ElasticityKind::StVK, false},
  {ElasticityKind::NeoHookean, false},
  {ElasticityKind::StableNeoHookean, false},
};

void ElasticityBatchedCompute::UpdateDeformationGradient(ConstRealBufferView f) {
  AX_THROW_IF_NULLPTR(deform_grad_, "deform_grad_ is not initialized.");
  AX_THROW_IF(f.Shape() != Dim3(dim_, dim_, n_cubature_points_),
              "f shape is not compatible with deform_grad_.");
  copy(deform_grad_->View(), f);
  UpdateDeformationGradient();
}

void ElasticityBatchedCompute::UpdateDeformationGradient() {
  if (auto it = requires_svd.find(kind_); it != requires_svd.end() && it->second) {
    if (device_ == BufferDevice::Host) {
      details::do_update_svd_host(deform_grad_->ConstView(), svd_u_->View(), svd_v_->View(),
                                  svd_s_->View());
    } else {
      AX_CUDA_CALL(details::do_update_svd_gpu(deform_grad_->ConstView(), svd_u_->View(),
                                              svd_v_->View(), svd_s_->View()));
    }
  }
}

ElasticityBatchedCompute::ElasticityBatchedCompute(size_t n_cubature_points, size_t dim,
                                                   BufferDevice device)
    : n_cubature_points_(n_cubature_points), dim_(dim), device_(device) {
  lame_ = create_buffer<Real>(device_, {2, n_cubature_points});
  energy_density_ = create_buffer<Real>(device_, {n_cubature_points});
  deform_grad_ = create_buffer<Real>(device_, {dim, dim, n_cubature_points});
  pk1_ = create_buffer<Real>(device_, {dim, dim, n_cubature_points});
  local_hessian_ = create_buffer<Real>(device_, {dim * dim, dim * dim, n_cubature_points});

  svd_u_ = create_buffer<Real>(device_, {dim, dim, n_cubature_points});
  svd_v_ = create_buffer<Real>(device_, {dim, dim, n_cubature_points});
  svd_s_ = create_buffer<Real>(device_, {dim, n_cubature_points});
}

void ElasticityBatchedCompute::UpdateEnergyDensity() {
  if (device_ == BufferDevice::Host) {
    details::do_update_energy_density_host(deform_grad_->ConstView(), lame_->ConstView(),
                                           svd_u_->ConstView(), svd_v_->ConstView(),
                                           svd_s_->ConstView(), energy_density_->View(), kind_);
  } else {
    AX_CUDA_CALL(details::do_update_energy_density_gpu(
        deform_grad_->ConstView(), lame_->ConstView(), svd_u_->ConstView(), svd_v_->ConstView(),
        svd_s_->ConstView(), energy_density_->View(), kind_));
  }
  is_energy_up_to_date_ = true;
}

void ElasticityBatchedCompute::UpdateGradient() {
  if (device_ == BufferDevice::Host) {
    details::do_update_gradient_host(deform_grad_->ConstView(), lame_->ConstView(),
                                     svd_u_->ConstView(), svd_v_->ConstView(), svd_s_->ConstView(),
                                     energy_density_->View(), pk1_->View(), kind_);
  } else {
    AX_CUDA_CALL(details::do_update_gradient_gpu(
        deform_grad_->ConstView(), lame_->ConstView(), svd_u_->ConstView(), svd_v_->ConstView(),
        svd_s_->ConstView(), energy_density_->View(), pk1_->View(), kind_));
  }
  is_energy_up_to_date_ = true;
  is_gradient_up_to_date_ = true;
}

void ElasticityBatchedCompute::UpdateHessian() {
  if (device_ == BufferDevice::Host) {
    details::do_update_hessian_host(
        deform_grad_->ConstView(), lame_->ConstView(), svd_u_->ConstView(), svd_v_->ConstView(),
        svd_s_->ConstView(), energy_density_->View(), pk1_->View(), local_hessian_->View(), kind_);
  } else {
    AX_CUDA_CALL(details::do_update_hessian_gpu(
        deform_grad_->ConstView(), lame_->ConstView(), svd_u_->ConstView(), svd_v_->ConstView(),
        svd_s_->ConstView(), energy_density_->View(), pk1_->View(), local_hessian_->View(), kind_));
  }
  is_energy_up_to_date_ = true;
  is_gradient_up_to_date_ = true;
  is_hessian_up_to_date_ = true;
}

void ElasticityBatchedCompute::SetElasitcityKind(ElasticityKind kind) {
  kind_ = kind;
  is_energy_up_to_date_ = false;
  is_gradient_up_to_date_ = false;
  is_hessian_up_to_date_ = false;
}

}  // namespace ax::fem