#pragma once

#include "ax/core/buffer/buffer_view.hpp"
#include "ax/fem/mesh.hpp"

namespace ax::fem {

AX_HOST_DEVICE AX_FORCE_INLINE math::RealMatrix<9, 12> ComputePFPx(
    const math::RealMatrix3& DmInv) {
  const Real m = DmInv(0, 0);
  const Real n = DmInv(0, 1);
  const Real o = DmInv(0, 2);
  const Real p = DmInv(1, 0);
  const Real q = DmInv(1, 1);
  const Real r = DmInv(1, 2);
  const Real s = DmInv(2, 0);
  const Real t = DmInv(2, 1);
  const Real u = DmInv(2, 2);
  const Real t1 = -m - p - s;
  const Real t2 = -n - q - t;
  const Real t3 = -o - r - u;
  math::RealMatrix<9, 12> PFPx;
  PFPx.setZero();
  PFPx(0, 0) = t1;
  PFPx(0, 3) = m;
  PFPx(0, 6) = p;
  PFPx(0, 9) = s;
  PFPx(1, 1) = t1;
  PFPx(1, 4) = m;
  PFPx(1, 7) = p;
  PFPx(1, 10) = s;
  PFPx(2, 2) = t1;
  PFPx(2, 5) = m;
  PFPx(2, 8) = p;
  PFPx(2, 11) = s;
  PFPx(3, 0) = t2;
  PFPx(3, 3) = n;
  PFPx(3, 6) = q;
  PFPx(3, 9) = t;
  PFPx(4, 1) = t2;
  PFPx(4, 4) = n;
  PFPx(4, 7) = q;
  PFPx(4, 10) = t;
  PFPx(5, 2) = t2;
  PFPx(5, 5) = n;
  PFPx(5, 8) = q;
  PFPx(5, 11) = t;
  PFPx(6, 0) = t3;
  PFPx(6, 3) = o;
  PFPx(6, 6) = r;
  PFPx(6, 9) = u;
  PFPx(7, 1) = t3;
  PFPx(7, 4) = o;
  PFPx(7, 7) = r;
  PFPx(7, 10) = u;
  PFPx(8, 2) = t3;
  PFPx(8, 5) = o;
  PFPx(8, 8) = r;
  PFPx(8, 11) = u;
  return PFPx;
}

// Auto generated code.
AX_HOST_DEVICE AX_FORCE_INLINE math::RealMatrix<4, 6> ComputePFPx(const math::RealMatrix2& DmInv) {
  const Real m = DmInv(0, 0);
  const Real n = DmInv(0, 1);
  const Real p = DmInv(1, 0);
  const Real q = DmInv(1, 1);
  const Real t1 = -m - p;
  const Real t2 = -n - q;
  math::RealMatrix<4, 6> PFPx;
  PFPx.setZero();
  PFPx(0, 0) = t1;
  PFPx(0, 2) = m;
  PFPx(0, 4) = p;
  PFPx(1, 1) = t1;
  PFPx(1, 3) = m;
  PFPx(1, 5) = p;
  PFPx(2, 0) = t2;
  PFPx(2, 2) = n;
  PFPx(2, 4) = q;
  PFPx(3, 1) = t2;
  PFPx(3, 3) = n;
  PFPx(3, 5) = q;
  return PFPx;
}

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