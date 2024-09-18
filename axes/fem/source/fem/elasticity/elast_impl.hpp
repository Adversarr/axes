#pragma once
#include "ax/fem/elasticity/compute.hpp"

namespace ax::fem::details {

void do_update_svd_host(
  ConstRealBufferView deform_grad, // deformation gradient
  RealBufferView svd_u, // u
  RealBufferView svd_v, // v
  RealBufferView svd_s // sigma
);

void do_update_svd_gpu(
  ConstRealBufferView deform_grad, // deformation gradient
  RealBufferView svd_u, // u
  RealBufferView svd_v, // v
  RealBufferView svd_s // sigma
);

void do_update_energy_density_host(
  ConstRealBufferView deform_grad, // deformation gradient
  ConstRealBufferView lame,  // lame coefficients
  ConstRealBufferView svd_u, // u
  ConstRealBufferView svd_v, // v
  ConstRealBufferView svd_s, // sigma
  RealBufferView energy_density,
  ElasticityKind kind
);

void do_update_energy_density_gpu(
  ConstRealBufferView deform_grad, // deformation gradient
  ConstRealBufferView lame,  // lame coefficients
  ConstRealBufferView svd_u, // u
  ConstRealBufferView svd_v, // v
  ConstRealBufferView svd_s, // sigma
  RealBufferView energy_density,
  ElasticityKind kind
);

void do_update_gradient_host(
  ConstRealBufferView deform_grad, // deformation gradient
  ConstRealBufferView lame,  // lame coefficients
  ConstRealBufferView svd_u, // u
  ConstRealBufferView svd_v, // v
  ConstRealBufferView svd_s, // sigma
  RealBufferView energy_density,
  RealBufferView pk1,
  ElasticityKind kind
);

void do_update_gradient_gpu(
  ConstRealBufferView deform_grad, // deformation gradient
  ConstRealBufferView lame,  // lame coefficients
  ConstRealBufferView svd_u, // u
  ConstRealBufferView svd_v, // v
  ConstRealBufferView svd_s, // sigma
  RealBufferView energy_density,
  RealBufferView pk1,
  ElasticityKind kind
);

void do_update_hessian_host(
  ConstRealBufferView deform_grad, // deformation gradient
  ConstRealBufferView lame,  // lame coefficients
  ConstRealBufferView svd_u, // u
  ConstRealBufferView svd_v, // v
  ConstRealBufferView svd_s, // sigma
  RealBufferView energy_density,
  RealBufferView pk1,
  RealBufferView local_hessian,
  ElasticityKind kind
);

void do_update_hessian_gpu(
  ConstRealBufferView deform_grad, // deformation gradient
  ConstRealBufferView lame,  // lame coefficients
  ConstRealBufferView svd_u, // u
  ConstRealBufferView svd_v, // v
  ConstRealBufferView svd_s, // sigma
  RealBufferView energy_density,
  RealBufferView pk1,
  RealBufferView local_hessian,
  ElasticityKind kind
);

}  // namespace ax::fem::details