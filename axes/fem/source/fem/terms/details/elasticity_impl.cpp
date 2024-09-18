#include "./elasticity_impl.hpp"

#include "ax/core/buffer/for_each.hpp"

namespace ax::fem {

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
    rest_volume(elem) = 0.5 * (vj - vi).dot(vk - vi);

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
    rest_volume(elem) = std::abs((vj - vi).cross(vk - vi).dot(vl - vi)) / 6.0;

    // Compute the inverse of the deformation gradient.
    math::RealMatrix3 dm;
    dm.col(0) = vj - vi;
    dm.col(1) = vk - vi;
    dm.col(2) = vl - vi;
    math::RealMatrix3 dm_inv_mat = dm.inverse().eval();
    MMapT(dminv.Offset(0, 0, elem)) = dm_inv_mat;

    // Compute the derivative of the deformation gradient wrt x.
    auto local_pfpx = ComputePFPx(dm_inv_mat);
    Eigen::Map<math::RealMatrix<9, 12>>(pfpx.Offset(0, 0, elem)) = local_pfpx;
  };

  size_t n_dof_per_vert = mesh.GetNumDOFPerVertex();
  if (n_dof_per_vert == 2) {
    for_each_indexed(Dim1{n_elem}, job2d);
  } else if (n_dof_per_vert == 3) {
    for_each_indexed(Dim1{n_elem}, job3d);
  } else {
    AX_THROW_RUNTIME_ERROR("Not implemented.");
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
    ds.col(1) = vk - vi + uj - ui;
    ds.col(2) = vl - vi + uj - ui;

    CMMapT dminv_elem(dminv.Offset(0, 0, elem));
    MMapT deformation_gradient_elem(deformation_gradient.Offset(0, 0, elem));
    deformation_gradient_elem = ds * dminv_elem;
  };

  size_t n_dof_per_vert = mesh.GetNumDOFPerVertex();
  if (n_dof_per_vert == 2) {
    for_each_indexed(Dim1{mesh.GetNumElements()}, job2d);
  } else if (n_dof_per_vert == 3) {
    for_each_indexed(Dim1{mesh.GetNumElements()}, job3d);
  } else {
    AX_THROW_RUNTIME_ERROR("Not implemented.");
  }
}

void compute_cubature_gradient_cpu(const Mesh& mesh, ConstRealBufferView grad,
                                   RealBufferView elem_grad) {
  auto e = mesh.GetElements()->ConstView();
  size_t n_vert_per_elem = mesh.GetNumVerticesPerElement();
  size_t n_dof = mesh.GetNumDOFPerVertex();
  auto job2d = [&](size_t elem) mutable {
    //////// TODO:
  };
}

void compute_cubature_hessian_cpu(const Mesh& mesh, ConstRealBufferView hessian,
                                  RealBufferView elem_hess);

}  // namespace ax::fem