#pragma once
#include "elast_impl.hpp"

#include "ax/core/buffer/for_each.hpp"
#include "ax/fem/elasticity/arap.hpp"
#include "ax/fem/elasticity/linear.hpp"
#include "ax/fem/elasticity/neohookean_bw.hpp"
#include "ax/fem/elasticity/stable_neohookean.hpp"
#include "ax/fem/elasticity/stvk.hpp"
#include "ax/math/decomp/svd/common.hpp"
#include "ax/math/decomp/svd/import_eigen.hpp"

namespace ax::fem::details {

template <int dim, template <int> class MaterialModel>
static void energy(ConstRealBufferView deform_grad, ConstRealBufferView lame,
                   ConstRealBufferView svd_u,  // u
                   ConstRealBufferView svd_v,  // v
                   ConstRealBufferView svd_s,  // sigma
                   RealBufferView energy_density) {
  size_t n_cubature_points = deform_grad.Shape().Z();
  par_for_each_indexed(Dim{n_cubature_points}, [&](size_t i) {
    MaterialModel<dim> model(lame(0, i), lame(1, i));
    using MapT = math::Map<const math::RealMatrix<dim, dim>>;
    using VMapT = math::Map<const math::RealVector<dim>>;
    MapT dg(deform_grad.Offset(0, 0, i));
    math::decomp::SvdResult<dim> svd_result;
    svd_result.U_ = MapT(svd_u.Offset(0, 0, i));
    svd_result.V_ = MapT(svd_v.Offset(0, 0, i));
    svd_result.sigma_ = VMapT(svd_s.Offset(0, i));
    energy_density(i) = model.Energy(dg, svd_result);
  });
}

template <int dim, template <int> class MaterialModel>
static void compute_pk1(ConstRealBufferView deform_grad,  // deformation gradient
                        ConstRealBufferView lame,         // lame coefficients
                        ConstRealBufferView svd_u,        // u
                        ConstRealBufferView svd_v,        // v
                        ConstRealBufferView svd_s,        // sigma
                        RealBufferView energy_density, RealBufferView pk1) {
  size_t n_cubature_points = deform_grad.Shape().Z();
  par_for_each_indexed(Dim{n_cubature_points}, [&](size_t i) {
    MaterialModel<dim> model(lame(0, i), lame(1, i));
    using MapT = math::Map<const math::RealMatrix<dim, dim>>;
    using VMapT = math::Map<const math::RealVector<dim>>;
    MapT dg(deform_grad.Offset(0, 0, i));
    math::decomp::SvdResult<dim> svd_result;
    svd_result.U_ = MapT(svd_u.Offset(0, 0, i));
    svd_result.V_ = MapT(svd_v.Offset(0, 0, i));
    svd_result.sigma_ = VMapT(svd_s.Offset(0, i));
    energy_density(i) = model.Energy(dg, svd_result);
    auto mapped = math::Map<math::RealMatrix<dim, dim>>(pk1.Offset(0, 0, i));
    mapped = model.Stress(dg, svd_result);
  });
}

template <int dim, template <int> class MaterialModel>
static void compute_hessian(ConstRealBufferView deform_grad,  // deformation gradient
                            ConstRealBufferView lame,         // lame coefficients
                            ConstRealBufferView svd_u,        // u
                            ConstRealBufferView svd_v,        // v
                            ConstRealBufferView svd_s,        // sigma
                            RealBufferView energy_density, RealBufferView pk1,
                            RealBufferView local_hessian) {
  size_t n_cubature_points = deform_grad.Shape().Z();
  par_for_each_indexed(Dim{n_cubature_points}, [&](size_t i) {
    MaterialModel<dim> model(lame(0, i), lame(1, i));
    using MapT = math::Map<const math::RealMatrix<dim, dim>>;
    using VMapT = math::Map<const math::RealVector<dim>>;
    MapT dg(deform_grad.Offset(0, 0, i));
    math::decomp::SvdResult<dim> svd_result;
    svd_result.U_ = MapT(svd_u.Offset(0, 0, i));
    svd_result.V_ = MapT(svd_v.Offset(0, 0, i));
    svd_result.sigma_ = VMapT(svd_s.Offset(0, i));
    energy_density(i) = model.Energy(dg, svd_result);
    auto mapped = math::Map<math::RealMatrix<dim, dim>>(pk1.Offset(0, 0, i));
    mapped = model.Stress(dg, svd_result);
    auto local = model.Hessian(dg, svd_result);

    for (size_t v = 0; v < dim; ++v) {
      for (size_t u = 0; u < dim; ++u) {                   // cache friendly
        size_t linear_id = i * (dim * dim) + u + v * dim;  // (u, v) = (Row, Col)
        // [dim, dim] matrix. The Hessian is (dim * dim, dim * dim), but we need the blockwise.
        math::Map<math::RealMatrix<dim, dim>> hessian(local_hessian.Offset(0, 0, linear_id));
        hessian = local.template block<dim, dim>(static_cast<Index>(u * dim),
                                                 static_cast<Index>(v * dim));
      }
    }
  });
}

template <int dim>
void do_update_svd_host_impl(ConstRealBufferView deform_grad, RealBufferView svd_u,
                             RealBufferView svd_v, RealBufferView svd_s) {
  size_t n_cubature_points = deform_grad.Shape().Z();
  par_for_each_indexed(Dim{n_cubature_points}, [&](size_t i) {
    using MapT = math::Map<const math::RealMatrix<dim, dim>>;
    using VMapT = math::Map<math::RealVector<dim>>;
    using MutMapT = math::Map<math::RealMatrix<dim, dim>>;
    MapT dg(deform_grad.Offset(0, 0, i));
    auto r = math::decomp::JacobiSvd<dim, Real>().Solve(dg);
    MutMapT(svd_u.Offset(0, 0, i)) = r.U_;
    MutMapT(svd_v.Offset(0, 0, i)) = r.V_;
    VMapT(svd_s.Offset(0, i)) = r.sigma_;
  });
}

void do_update_svd_host(ConstRealBufferView deform_grad, RealBufferView svd_u, RealBufferView svd_v,
                        RealBufferView svd_s) {
  size_t dim = deform_grad.Shape().X();
  if (dim == 2) {
    do_update_svd_host_impl<2>(deform_grad, svd_u, svd_v, svd_s);
  } else if (dim == 3) {
    do_update_svd_host_impl<3>(deform_grad, svd_u, svd_v, svd_s);
  } else {
    AX_THROW_RUNTIME_ERROR("Unsupported dimension.");
  }
}

void do_update_energy_density_host(ConstRealBufferView deform_grad,  // deformation gradient
                                   ConstRealBufferView lame,         // lame coefficients
                                   ConstRealBufferView svd_u,        // u
                                   ConstRealBufferView svd_v,        // v
                                   ConstRealBufferView svd_s,        // sigma
                                   RealBufferView energy_density, ElasticityKind kind) {
  size_t dim = deform_grad.Shape().X();
  if (dim == 2) {
    switch (kind) {
      case ElasticityKind::Linear:
        energy<2, elasticity::Linear>(deform_grad, lame, svd_u, svd_v, svd_s, energy_density);
        break;
      case ElasticityKind::IsotropicARAP:
        energy<2, elasticity::IsotropicARAP>(deform_grad, lame, svd_u, svd_v, svd_s,
                                             energy_density);
        break;
      case ElasticityKind::StVK:
        energy<2, elasticity::StVK>(deform_grad, lame, svd_u, svd_v, svd_s, energy_density);
        break;
      case ElasticityKind::NeoHookean:
        energy<2, elasticity::NeoHookeanBW>(deform_grad, lame, svd_u, svd_v, svd_s, energy_density);
        break;
      case ElasticityKind::StableNeoHookean:
        energy<2, elasticity::StableNeoHookean>(deform_grad, lame, svd_u, svd_v, svd_s,
                                                energy_density);
        break;
      default:
        AX_THROW_RUNTIME_ERROR("Unsupported elasticity kind.");
    }
  } else {
    switch (kind) {
      case ElasticityKind::Linear:
        energy<3, elasticity::Linear>(deform_grad, lame, svd_u, svd_v, svd_s, energy_density);
        break;
      case ElasticityKind::IsotropicARAP:
        energy<3, elasticity::IsotropicARAP>(deform_grad, lame, svd_u, svd_v, svd_s,
                                             energy_density);
        break;
      case ElasticityKind::StVK:
        energy<3, elasticity::StVK>(deform_grad, lame, svd_u, svd_v, svd_s, energy_density);
        break;
      case ElasticityKind::NeoHookean:
        energy<3, elasticity::NeoHookeanBW>(deform_grad, lame, svd_u, svd_v, svd_s, energy_density);
        break;
      case ElasticityKind::StableNeoHookean:
        energy<3, elasticity::StableNeoHookean>(deform_grad, lame, svd_u, svd_v, svd_s,
                                                energy_density);
        break;
      default:
        AX_THROW_RUNTIME_ERROR("Unsupported elasticity kind.");
    }
  }
}

void do_update_gradient_host(ConstRealBufferView deform_grad,  // deformation gradient
                             ConstRealBufferView lame,         // lame coefficients
                             ConstRealBufferView svd_u,        // u
                             ConstRealBufferView svd_v,        // v
                             ConstRealBufferView svd_s,        // sigma
                             RealBufferView energy_density, RealBufferView pk1,
                             ElasticityKind kind) {
  size_t dim = deform_grad.Shape().X();
  if (dim == 2) {
    switch (kind) {
      case ElasticityKind::Linear:
        compute_pk1<2, elasticity::Linear>(deform_grad, lame, svd_u, svd_v, svd_s, energy_density,
                                           pk1);
        break;
      case ElasticityKind::IsotropicARAP:
        compute_pk1<2, elasticity::IsotropicARAP>(deform_grad, lame, svd_u, svd_v, svd_s,
                                                  energy_density, pk1);
        break;
      case ElasticityKind::StVK:
        compute_pk1<2, elasticity::StVK>(deform_grad, lame, svd_u, svd_v, svd_s, energy_density,
                                         pk1);
        break;
      case ElasticityKind::NeoHookean:
        compute_pk1<2, elasticity::NeoHookeanBW>(deform_grad, lame, svd_u, svd_v, svd_s,
                                                 energy_density, pk1);
        break;
      case ElasticityKind::StableNeoHookean:
        compute_pk1<2, elasticity::StableNeoHookean>(deform_grad, lame, svd_u, svd_v, svd_s,
                                                     energy_density, pk1);
        break;
      default:
        AX_THROW_RUNTIME_ERROR("Unsupported elasticity kind.");
    }
  } else {
    switch (kind) {
      case ElasticityKind::Linear:
        compute_pk1<3, elasticity::Linear>(deform_grad, lame, svd_u, svd_v, svd_s, energy_density,
                                           pk1);
        break;
      case ElasticityKind::IsotropicARAP:
        compute_pk1<3, elasticity::IsotropicARAP>(deform_grad, lame, svd_u, svd_v, svd_s,
                                                  energy_density, pk1);
        break;
      case ElasticityKind::StVK:
        compute_pk1<3, elasticity::StVK>(deform_grad, lame, svd_u, svd_v, svd_s, energy_density,
                                         pk1);
        break;
      case ElasticityKind::NeoHookean:
        compute_pk1<3, elasticity::NeoHookeanBW>(deform_grad, lame, svd_u, svd_v, svd_s,
                                                 energy_density, pk1);
        break;
      case ElasticityKind::StableNeoHookean:
        compute_pk1<3, elasticity::StableNeoHookean>(deform_grad, lame, svd_u, svd_v, svd_s,
                                                     energy_density, pk1);
        break;
      default:
        AX_THROW_RUNTIME_ERROR("Unsupported elasticity kind.");
    }
  }
}

void do_update_hessian_host(ConstRealBufferView deform_grad,  // deformation gradient
                            ConstRealBufferView lame,         // lame coefficients
                            ConstRealBufferView svd_u,        // u
                            ConstRealBufferView svd_v,        // v
                            ConstRealBufferView svd_s,        // sigma
                            RealBufferView energy_density, RealBufferView pk1,
                            RealBufferView local_hessian, ElasticityKind kind) {
  size_t dim = deform_grad.Shape().X();
  if (dim == 2) {
    switch (kind) {
      case ElasticityKind::Linear:
        compute_hessian<2, elasticity::Linear>(deform_grad, lame, svd_u, svd_v, svd_s,
                                               energy_density, pk1, local_hessian);
        break;
      case ElasticityKind::IsotropicARAP:
        compute_hessian<2, elasticity::IsotropicARAP>(deform_grad, lame, svd_u, svd_v, svd_s,
                                                      energy_density, pk1, local_hessian);
        break;
      case ElasticityKind::StVK:
        compute_hessian<2, elasticity::StVK>(deform_grad, lame, svd_u, svd_v, svd_s, energy_density,
                                             pk1, local_hessian);
        break;
      case ElasticityKind::NeoHookean:
        compute_hessian<2, elasticity::NeoHookeanBW>(deform_grad, lame, svd_u, svd_v, svd_s,
                                                     energy_density, pk1, local_hessian);
        break;
      case ElasticityKind::StableNeoHookean:
        compute_hessian<2, elasticity::StableNeoHookean>(deform_grad, lame, svd_u, svd_v, svd_s,
                                                         energy_density, pk1, local_hessian);
        break;
      default:
        AX_THROW_RUNTIME_ERROR("Unsupported elasticity kind.");
    }
  } else {
    switch (kind) {
      case ElasticityKind::Linear:
        compute_hessian<3, elasticity::Linear>(deform_grad, lame, svd_u, svd_v, svd_s,
                                               energy_density, pk1, local_hessian);
        break;
      case ElasticityKind::IsotropicARAP:
        compute_hessian<3, elasticity::IsotropicARAP>(deform_grad, lame, svd_u, svd_v, svd_s,
                                                      energy_density, pk1, local_hessian);
        break;
      case ElasticityKind::StVK:
        compute_hessian<3, elasticity::StVK>(deform_grad, lame, svd_u, svd_v, svd_s, energy_density,
                                             pk1, local_hessian);
        break;
      case ElasticityKind::NeoHookean:
        compute_hessian<3, elasticity::NeoHookeanBW>(deform_grad, lame, svd_u, svd_v, svd_s,
                                                     energy_density, pk1, local_hessian);
        break;
      case ElasticityKind::StableNeoHookean:
        compute_hessian<3, elasticity::StableNeoHookean>(deform_grad, lame, svd_u, svd_v, svd_s,
                                                         energy_density, pk1, local_hessian);
        break;
      default:
        AX_THROW_RUNTIME_ERROR("Unsupported elasticity kind.");
    }
  }
}

}  // namespace ax::fem::details