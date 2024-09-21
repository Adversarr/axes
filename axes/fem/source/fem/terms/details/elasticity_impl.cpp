#include "./elasticity_impl.hpp"

#include "ax/core/buffer/for_each.hpp"
#include "ax/math/common.hpp"
#include <Eigen/Geometry>

namespace ax::fem {

static AX_FORCE_INLINE  math::RealMatrix<9, 12> ComputePFPx(const math::RealMatrix3& DmInv) {
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
  math::RealMatrix<9, 12> pf_px;
  pf_px.setZero();
  pf_px(0, 0) = t1;
  pf_px(0, 3) = m;
  pf_px(0, 6) = p;
  pf_px(0, 9) = s;
  pf_px(1, 1) = t1;
  pf_px(1, 4) = m;
  pf_px(1, 7) = p;
  pf_px(1, 10) = s;
  pf_px(2, 2) = t1;
  pf_px(2, 5) = m;
  pf_px(2, 8) = p;
  pf_px(2, 11) = s;
  pf_px(3, 0) = t2;
  pf_px(3, 3) = n;
  pf_px(3, 6) = q;
  pf_px(3, 9) = t;
  pf_px(4, 1) = t2;
  pf_px(4, 4) = n;
  pf_px(4, 7) = q;
  pf_px(4, 10) = t;
  pf_px(5, 2) = t2;
  pf_px(5, 5) = n;
  pf_px(5, 8) = q;
  pf_px(5, 11) = t;
  pf_px(6, 0) = t3;
  pf_px(6, 3) = o;
  pf_px(6, 6) = r;
  pf_px(6, 9) = u;
  pf_px(7, 1) = t3;
  pf_px(7, 4) = o;
  pf_px(7, 7) = r;
  pf_px(7, 10) = u;
  pf_px(8, 2) = t3;
  pf_px(8, 5) = o;
  pf_px(8, 8) = r;
  pf_px(8, 11) = u;
  return pf_px;
}

// Auto generated code.
static AX_FORCE_INLINE  math::RealMatrix<4, 6> ComputePFPx(const math::RealMatrix2& DmInv) {
  const Real m = DmInv(0, 0);
  const Real n = DmInv(0, 1);
  const Real p = DmInv(1, 0);
  const Real q = DmInv(1, 1);
  const Real t1 = -m - p;
  const Real t2 = -n - q;
  math::RealMatrix<4, 6> pf_px;
  pf_px.setZero();
  pf_px(0, 0) = t1;
  pf_px(0, 2) = m;
  pf_px(0, 4) = p;
  pf_px(1, 1) = t1;
  pf_px(1, 3) = m;
  pf_px(1, 5) = p;
  pf_px(2, 0) = t2;
  pf_px(2, 2) = n;
  pf_px(2, 4) = q;
  pf_px(3, 1) = t2;
  pf_px(3, 3) = n;
  pf_px(3, 5) = q;
  return pf_px;
}

void compute_static_data_cpu(const Mesh& mesh, ConstRealBufferView rest_pose,
                             RealBufferView rest_volume, RealBufferView dminv,
                             RealBufferView pfpx) {
  auto e = make_view(mesh.GetElements());
  size_t n_elem = mesh.GetNumElements();

  auto job2d = [&](size_t elem) mutable {
    auto i = e(0, elem);
    auto j = e(1, elem);
    auto k = e(2, elem);
    using CVMapT = Eigen::Map<const math::RealVector2>;
    using MMapT = Eigen::Map<math::RealMatrix2>;
    CVMapT vi(rest_pose.Offset(0, i)), vj(rest_pose.Offset(0, j)), vk(rest_pose.Offset(0, k));

    // Compute the rest volume.
    rest_volume(elem) = abs(0.5 * (vj - vi).dot(vk - vi));

    // Compute the inverse of the deformation gradient.
    math::RealMatrix2 F;
    F.col(0) = vj - vi;
    F.col(1) = vk - vi;
    math::RealMatrix2 invF = F.inverse().eval();
    MMapT(dminv.Offset(0, 0, elem)) = invF;

    // Compute the derivative of the deformation gradient wrt x.
    auto local_pfpx = ComputePFPx(invF);
    Eigen::Map<math::RealMatrix<4, 6>>(pfpx.Offset(0, 0, elem)) = local_pfpx;
  };

  auto job3d = [&](size_t elem) mutable {
    auto i = e(0, elem);
    auto j = e(1, elem);
    auto k = e(2, elem);
    auto l = e(3, elem);
    using CVMapT = Eigen::Map<const math::RealVector3>;
    using MMapT = Eigen::Map<math::RealMatrix3>;
    CVMapT vi(rest_pose.Offset(0, i)), vj(rest_pose.Offset(0, j)), vk(rest_pose.Offset(0, k)),
        vl(rest_pose.Offset(0, l));

    // Compute the rest volume.

    // Compute the inverse of the deformation gradient.
    math::RealMatrix3 dm;
    dm.col(0) = vj - vi;
    dm.col(1) = vk - vi;
    dm.col(2) = vl - vi;
    rest_volume(elem) = std::abs(dm.determinant() / 6.0);
    math::RealMatrix3 dm_inv_mat = dm.inverse().eval();
    MMapT(dminv.Offset(0, 0, elem)) = dm_inv_mat;

    // Compute the derivative of the deformation gradient wrt x.
    auto local_pfpx = ComputePFPx(dm_inv_mat);
    Eigen::Map<math::RealMatrix<9, 12>>(pfpx.Offset(0, 0, elem)) = local_pfpx;
  };

  size_t n_dof_per_vert = mesh.GetNumDOFPerVertex();
  if (n_dof_per_vert == 2) {
    par_for_each_indexed(Dim1{n_elem}, job2d);
  } else if (n_dof_per_vert == 3) {
    par_for_each_indexed(Dim1{n_elem}, job3d);
  } else {
    AX_NOT_IMPLEMENTED();
  }
}

void compute_deformation_gradient_cpu(const Mesh& mesh, ConstRealBufferView dminv,
                                      ConstRealBufferView u, RealBufferView deformation_gradient) {
  auto vert = mesh.GetVertices()->ConstView();
  auto e = mesh.GetElements()->ConstView();

  auto job2d = [&](size_t elem) mutable {
    auto i = e(0, elem);
    auto j = e(1, elem);
    auto k = e(2, elem);
    using CVMapT = Eigen::Map<const math::RealVector2>;
    using CMMapT = Eigen::Map<const math::RealMatrix2>;
    using MMapT = Eigen::Map<math::RealMatrix2>;
    CVMapT vi(vert.Offset(0, i)), vj(vert.Offset(0, j)), vk(vert.Offset(0, k));
    CVMapT ui(u.Offset(0, i)), uj(u.Offset(0, j)), uk(u.Offset(0, k));
    // Compute the deformation gradient.
    math::RealMatrix2 ds;
    ds.col(0) = vj - vi + uj - ui;
    ds.col(1) = vk - vi + uk - ui;

    CMMapT dminv_elem(dminv.Offset(0, 0, elem));
    MMapT deformation_gradient_elem(deformation_gradient.Offset(0, 0, elem));
    deformation_gradient_elem = ds * dminv_elem;
  };

  auto job3d = [&](size_t elem) mutable {
    auto i = e(0, elem);
    auto j = e(1, elem);
    auto k = e(2, elem);
    auto l = e(3, elem);
    using CVMapT = Eigen::Map<const math::RealVector3>;
    using CMMapT = Eigen::Map<const math::RealMatrix3>;
    using MMapT = Eigen::Map<math::RealMatrix3>;
    CVMapT vi(vert.Offset(0, i)), vj(vert.Offset(0, j)), vk(vert.Offset(0, k)),
        vl(vert.Offset(0, l));
    CVMapT ui(u.Offset(0, i)), uj(u.Offset(0, j)), uk(u.Offset(0, k)), ul(u.Offset(0, l));
    // Compute the deformation gradient.
    math::RealMatrix3 ds;
    ds.col(0) = vj - vi + uj - ui;
    ds.col(1) = vk - vi + uk - ui;
    ds.col(2) = vl - vi + ul - ui;

    CMMapT dminv_elem(dminv.Offset(0, 0, elem));
    MMapT deformation_gradient_elem(deformation_gradient.Offset(0, 0, elem));
    deformation_gradient_elem = ds * dminv_elem;
  };

  size_t n_dof_per_vert = mesh.GetNumDOFPerVertex();
  if (n_dof_per_vert == 2) {
    par_for_each_indexed(Dim1{mesh.GetNumElements()}, job2d);
  } else if (n_dof_per_vert == 3) {
    par_for_each_indexed(Dim1{mesh.GetNumElements()}, job3d);
  } else {
    AX_NOT_IMPLEMENTED();
  }
}

void compute_cubature_gradient_cpu(const Mesh& mesh, ConstRealBufferView grad,
                                   ConstRealBufferView dminv, RealBufferView elem_grad) {
  size_t n_dof = mesh.GetNumDOFPerVertex();
  auto job2d = [&](size_t elem) mutable {
    using CMMapT = Eigen::Map<const math::RealMatrix2>;
    using MMapT = Eigen::Map<math::RealMatrix<2, 3>>;
    CMMapT grad_elem(grad.Offset(0, 0, elem));           // 2x2.
    CMMapT dminv_elem(dminv.Offset(0, 0, elem));         // 2x2.
    MMapT elem_grad_elem(elem_grad.Offset(0, 0, elem));  // 2x3.
    math::RealMatrix2 grad_elem_x = grad_elem * dminv_elem.transpose();
    elem_grad_elem.col(0) = -(grad_elem_x.col(0) + grad_elem_x.col(1));
    elem_grad_elem.col(1) = grad_elem_x.col(0);
    elem_grad_elem.col(2) = grad_elem_x.col(1);
  };

  auto job3d = [&](size_t elem) mutable {
    using CMMapT = Eigen::Map<const math::RealMatrix3>;
    using MMapT = Eigen::Map<math::RealMatrix<3, 4>>;
    CMMapT grad_elem(grad.Offset(0, 0, elem));           // 3x3.
    CMMapT dminv_elem(dminv.Offset(0, 0, elem));         // 3x3.
    MMapT elem_grad_elem(elem_grad.Offset(0, 0, elem));  // 3x4.
    math::RealMatrix3 grad_elem_x = grad_elem * dminv_elem.transpose();
    elem_grad_elem.col(0) = -(grad_elem_x.col(0) + grad_elem_x.col(1) + grad_elem_x.col(2));
    elem_grad_elem.col(1) = grad_elem_x.col(0);
    elem_grad_elem.col(2) = grad_elem_x.col(1);
    elem_grad_elem.col(3) = grad_elem_x.col(2);
  };

  if (n_dof == 2) {
    par_for_each_indexed(Dim1{mesh.GetNumElements()}, job2d);
  } else if (n_dof == 3) {
    par_for_each_indexed(Dim1{mesh.GetNumElements()}, job3d);
  } else {
    AX_NOT_IMPLEMENTED();
  }
}

void compute_cubature_hessian_cpu(const Mesh& mesh, ConstRealBufferView hessian,
                                  ConstRealBufferView pfpx, RealBufferView elem_hess) {
  size_t n_dof = mesh.GetNumDOFPerVertex();

  auto job2d = [&](size_t elem) mutable {
    using CMMapT = Eigen::Map<const math::RealMatrix4>;
    using CMMapT46 = Eigen::Map<const math::RealMatrix<4, 6>>;
    CMMapT hessian_elem(hessian.Offset(0, 0, elem));           // 2x2.
    CMMapT46 pfpx_elem(pfpx.Offset(0, 0, elem));               // 4x6.
    using MMapT = Eigen::Map<math::RealMatrix<2, 2>>;

    math::RealMatrix<6, 6> local_hessian = pfpx_elem.transpose() * hessian_elem * pfpx_elem;
    for (size_t j = 0; j < 3; ++j) {
      for (size_t i = 0; i < 3; ++i) {
        size_t linear = elem * 9 + j * 3 + i;
        MMapT elem_hess_elem(elem_hess.Offset(0, 0, linear));
        elem_hess_elem
            = local_hessian.block<2, 2>(static_cast<Index>(i * 2), static_cast<Index>(j * 2));
      }
    }
  };

  auto job3d = [&](size_t elem) mutable {
    using CMMapT = Eigen::Map<const math::RealMatrix<9, 9>>;
    using CMMapT912 = Eigen::Map<const math::RealMatrix<9, 12>>;
    CMMapT hessian_elem(hessian.Offset(0, 0, elem));           // 9x9
    CMMapT912 pfpx_elem(pfpx.Offset(0, 0, elem));              // 9x12.
    using MMapT = Eigen::Map<math::RealMatrix<3, 3>>;

    math::RealMatrix<12, 12> local_hessian = pfpx_elem.transpose() * hessian_elem * pfpx_elem;
    for (size_t j = 0; j < 4; ++j) {
      for (size_t i = 0; i < 4; ++i) {
        size_t linear = elem * 16 + j * 4 + i;
        MMapT elem_hess_elem(elem_hess.Offset(0, 0, linear));
        elem_hess_elem
            = local_hessian.block<3, 3>(static_cast<Index>(i * 3), static_cast<Index>(j * 3));
      }
    }
  };

  if (n_dof == 2) {
    par_for_each_indexed(Dim1{mesh.GetNumElements()}, job2d);
  } else if (n_dof == 3) {
    par_for_each_indexed(Dim1{mesh.GetNumElements()}, job3d);
  } else {
    AX_NOT_IMPLEMENTED();
  }
}

}  // namespace ax::fem