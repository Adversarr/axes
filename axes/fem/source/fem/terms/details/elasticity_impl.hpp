#pragma once

#include "ax/core/buffer/buffer_view.hpp"
#include "ax/fem/mesh.hpp"

namespace ax::fem {

void compute_static_data_cpu(const Mesh& mesh, ConstRealBufferView rest_pose,
                             RealBufferView rest_volume, RealBufferView dminv, RealBufferView pfpx);

void compute_deformation_gradient_cpu(const Mesh& mesh, ConstRealBufferView dminv,
                                      ConstRealBufferView u, RealBufferView deformation_gradient);
void compute_cubature_gradient_cpu(const Mesh& mesh, ConstRealBufferView grad,
                                   ConstRealBufferView dminv, RealBufferView elem_grad);

void compute_cubature_hessian_cpu(const Mesh& mesh, ConstRealBufferView hessian,
                                  ConstRealBufferView pfpx, RealBufferView elem_hess);

void compute_static_data_gpu(const Mesh& mesh, ConstRealBufferView rest_pose,
                             RealBufferView rest_volume, RealBufferView dminv, RealBufferView pfpx);

void compute_deformation_gradient_gpu(const Mesh& mesh, ConstRealBufferView dminv,
                                      ConstRealBufferView u, RealBufferView deformation_gradient);

void compute_cubature_gradient_gpu(const Mesh& mesh, ConstRealBufferView grad,
                                   ConstRealBufferView dminv, RealBufferView elem_grad);

void compute_cubature_hessian_gpu(const Mesh& mesh, ConstRealBufferView hessian,
                                  ConstRealBufferView pfpx, RealBufferView elem_hess);

}  // namespace ax::fem