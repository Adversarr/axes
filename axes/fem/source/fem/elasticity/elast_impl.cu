#include "ax/core/excepts.hpp"
#include "ax/fem/elasticity/arap.hpp"
#include "ax/fem/elasticity/linear.hpp"
#include "ax/fem/elasticity/neohookean_bw.hpp"
#include "ax/fem/elasticity/stable_neohookean.hpp"
#include "ax/fem/elasticity/stvk.hpp"
#include "ax/math/common.hpp"
#include "ax/math/decomp/svd/common.hpp"
#include "ax/math/decomp/svd/svd_cuda.cuh"
#include "elast_impl.hpp"

namespace ax::fem::details {

#define BLOCK_SIZE 64

template <int dim, template <int> class Model>
__global__ void
compute_energy(ConstRealBufferView deform_grad, // deformation gradient
               ConstRealBufferView lame,        // lame coefficients
               ConstRealBufferView svd_u,       // u
               ConstRealBufferView svd_v,       // v
               ConstRealBufferView svd_s,       // sigma
               RealBufferView energy_density, size_t n_cube) {
  size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_cube)
    return;

  Model<dim> model(lame(0, i), lame(1, i));
  using MapT = math::Map<const math::RealMatrix<dim, dim>>;
  using VMapT = math::Map<const math::RealVector<dim>>;
  MapT dg(deform_grad.Offset(0, 0, i));
  math::decomp::SvdResult<dim> svd_result;
  svd_result.U_ = MapT(svd_u.Offset(0, 0, i));
  svd_result.V_ = MapT(svd_v.Offset(0, 0, i));
  svd_result.sigma_ = VMapT(svd_s.Offset(0, i));
  energy_density(i) = model.Energy(dg, svd_result);
}

template <int dim, template <int> class Model>
__global__ void
compute_pk1(ConstRealBufferView deform_grad, // deformation gradient
            ConstRealBufferView lame,        // lame coefficients
            ConstRealBufferView svd_u,       // u
            ConstRealBufferView svd_v,       // v
            ConstRealBufferView svd_s,       // sigma
            RealBufferView energy_density, RealBufferView pk1, size_t n_cube) {
  size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_cube)
    return;

  Model<dim> model(lame(0, i), lame(1, i));
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
}

template <int dim, template <int> class Model>
__global__ void
compute_hessian(ConstRealBufferView deform_grad, // deformation gradient
                ConstRealBufferView lame,        // lame coefficients
                ConstRealBufferView svd_u,       // u
                ConstRealBufferView svd_v,       // v
                ConstRealBufferView svd_s,       // sigma
                RealBufferView energy_density, RealBufferView pk1,
                RealBufferView local_hessian, size_t n_cube) {
  size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_cube)
    return;

  Model<dim> model(lame(0, i), lame(1, i));
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

  math::Map<math::RealMatrix<dim * dim, dim * dim>>(
      local_hessian.Offset(0, 0, i)) = model.Hessian(dg, svd_result);
}
static inline unsigned int up_div(unsigned int a, unsigned int b) {
  return (a + b - 1) / b;
}

void do_update_energy_density_gpu(
    ConstRealBufferView deform_grad, // deformation gradient
    ConstRealBufferView lame,        // lame coefficients
    ConstRealBufferView svd_u,       // u
    ConstRealBufferView svd_v,       // v
    ConstRealBufferView svd_s,       // sigma
    RealBufferView energy_density, ElasticityKind kind) {
  size_t dim = deform_grad.Shape().X();
  size_t n_cube = deform_grad.Shape().Z();
  unsigned int n_thread = BLOCK_SIZE;
  unsigned int n_block = up_div(static_cast<unsigned int>(n_cube), n_thread);

  if (dim == 2) {
    if (kind == ElasticityKind::Linear) {
      compute_energy<2, elasticity::Linear><<<n_block, n_thread>>>(
          deform_grad, lame, svd_u, svd_v, svd_s, energy_density, n_cube);
    } else if (kind == ElasticityKind::NeoHookean) {
      compute_energy<2, elasticity::NeoHookeanBW><<<n_block, n_thread>>>(
          deform_grad, lame, svd_u, svd_v, svd_s, energy_density, n_cube);
    } else if (kind == ElasticityKind::StVK) {
      compute_energy<2, elasticity::StVK><<<n_block, n_thread>>>(
          deform_grad, lame, svd_u, svd_v, svd_s, energy_density, n_cube);
    } else if (kind == ElasticityKind::StableNeoHookean) {
      compute_energy<2, elasticity::StableNeoHookean><<<n_block, n_thread>>>(
          deform_grad, lame, svd_u, svd_v, svd_s, energy_density, n_cube);
    } else if (kind == ElasticityKind::IsotropicARAP) {
      compute_energy<2, elasticity::IsotropicARAP><<<n_block, n_thread>>>(
          deform_grad, lame, svd_u, svd_v, svd_s, energy_density, n_cube);
    } else {
      AX_THROW_RUNTIME_ERROR("Unsupported elasticity model");
    }
  } else {
    if (kind == ElasticityKind::Linear) {
      compute_energy<3, elasticity::Linear><<<n_block, n_thread>>>(
          deform_grad, lame, svd_u, svd_v, svd_s, energy_density, n_cube);
    } else if (kind == ElasticityKind::NeoHookean) {
      compute_energy<3, elasticity::NeoHookeanBW><<<n_block, n_thread>>>(
          deform_grad, lame, svd_u, svd_v, svd_s, energy_density, n_cube);
    } else if (kind == ElasticityKind::StVK) {
      compute_energy<3, elasticity::StVK><<<n_block, n_thread>>>(
          deform_grad, lame, svd_u, svd_v, svd_s, energy_density, n_cube);
    } else if (kind == ElasticityKind::StableNeoHookean) {
      compute_energy<3, elasticity::StableNeoHookean><<<n_block, n_thread>>>(
          deform_grad, lame, svd_u, svd_v, svd_s, energy_density, n_cube);
    } else if (kind == ElasticityKind::IsotropicARAP) {
      compute_energy<3, elasticity::IsotropicARAP><<<n_block, n_thread>>>(
          deform_grad, lame, svd_u, svd_v, svd_s, energy_density, n_cube);
    } else {
      AX_THROW_RUNTIME_ERROR("Unsupported elasticity model");
    }
  }
}

void do_update_gradient_gpu(
    ConstRealBufferView deform_grad, // deformation gradient
    ConstRealBufferView lame,        // lame coefficients
    ConstRealBufferView svd_u,       // u
    ConstRealBufferView svd_v,       // v
    ConstRealBufferView svd_s,       // sigma
    RealBufferView energy_density, RealBufferView pk1, ElasticityKind kind) {
  size_t dim = deform_grad.Shape().X();
  size_t n_cube = deform_grad.Shape().Z();

  unsigned int n_thread = BLOCK_SIZE;
  unsigned int n_block = up_div(static_cast<unsigned int>(n_cube), n_thread);

  if (dim == 2) {
    if (kind == ElasticityKind::Linear) {
      compute_pk1<2, elasticity::Linear><<<n_block, n_thread>>>(
          deform_grad, lame, svd_u, svd_v, svd_s, energy_density, pk1, n_cube);
    } else if (kind == ElasticityKind::NeoHookean) {
      compute_pk1<2, elasticity::NeoHookeanBW><<<n_block, n_thread>>>(
          deform_grad, lame, svd_u, svd_v, svd_s, energy_density, pk1, n_cube);
    } else if (kind == ElasticityKind::StVK) {
      compute_pk1<2, elasticity::StVK><<<n_block, n_thread>>>(
          deform_grad, lame, svd_u, svd_v, svd_s, energy_density, pk1, n_cube);
    } else if (kind == ElasticityKind::StableNeoHookean) {
      compute_pk1<2, elasticity::StableNeoHookean><<<n_block, n_thread>>>(
          deform_grad, lame, svd_u, svd_v, svd_s, energy_density, pk1, n_cube);
    } else if (kind == ElasticityKind::IsotropicARAP) {
      compute_pk1<2, elasticity::IsotropicARAP><<<n_block, n_thread>>>(
          deform_grad, lame, svd_u, svd_v, svd_s, energy_density, pk1, n_cube);
    } else {
      AX_THROW_RUNTIME_ERROR("Unsupported elasticity model");
    }
  } else {
    if (kind == ElasticityKind::Linear) {
      compute_pk1<3, elasticity::Linear><<<n_block, n_thread>>>(
          deform_grad, lame, svd_u, svd_v, svd_s, energy_density, pk1, n_cube);
    } else if (kind == ElasticityKind::NeoHookean) {
      compute_pk1<3, elasticity::NeoHookeanBW><<<n_block, n_thread>>>(
          deform_grad, lame, svd_u, svd_v, svd_s, energy_density, pk1, n_cube);
    } else if (kind == ElasticityKind::StVK) {
      compute_pk1<3, elasticity::StVK><<<n_block, n_thread>>>(
          deform_grad, lame, svd_u, svd_v, svd_s, energy_density, pk1, n_cube);
    } else if (kind == ElasticityKind::StableNeoHookean) {
      compute_pk1<3, elasticity::StableNeoHookean><<<n_block, n_thread>>>(
          deform_grad, lame, svd_u, svd_v, svd_s, energy_density, pk1, n_cube);
    } else if (kind == ElasticityKind::IsotropicARAP) {
      compute_pk1<3, elasticity::IsotropicARAP><<<n_block, n_thread>>>(
          deform_grad, lame, svd_u, svd_v, svd_s, energy_density, pk1, n_cube);
    } else {
      AX_THROW_RUNTIME_ERROR("Unsupported elasticity model");
    }
  }
}

void do_update_hessian_gpu(
    ConstRealBufferView deform_grad, // deformation gradient
    ConstRealBufferView lame,        // lame coefficients
    ConstRealBufferView svd_u,       // u
    ConstRealBufferView svd_v,       // v
    ConstRealBufferView svd_s,       // sigma
    RealBufferView energy_density, RealBufferView pk1,
    RealBufferView local_hessian, ElasticityKind kind) {
  size_t dim = deform_grad.Shape().X();
  size_t n_cube = deform_grad.Shape().Z();

  unsigned int n_thread = BLOCK_SIZE;
  unsigned int n_block = up_div(static_cast<unsigned int>(n_cube), n_thread);

  if (dim == 2) {
    if (kind == ElasticityKind::Linear) {
      compute_hessian<2, elasticity::Linear>
          <<<n_block, n_thread>>>(deform_grad, lame, svd_u, svd_v, svd_s,
                                  energy_density, pk1, local_hessian, n_cube);
    } else if (kind == ElasticityKind::NeoHookean) {
      compute_hessian<2, elasticity::NeoHookeanBW>
          <<<n_block, n_thread>>>(deform_grad, lame, svd_u, svd_v, svd_s,
                                  energy_density, pk1, local_hessian, n_cube);
    } else if (kind == ElasticityKind::StVK) {
      compute_hessian<2, elasticity::StVK>
          <<<n_block, n_thread>>>(deform_grad, lame, svd_u, svd_v, svd_s,
                                  energy_density, pk1, local_hessian, n_cube);
    } else if (kind == ElasticityKind::StableNeoHookean) {
      compute_hessian<2, elasticity::StableNeoHookean>
          <<<n_block, n_thread>>>(deform_grad, lame, svd_u, svd_v, svd_s,
                                  energy_density, pk1, local_hessian, n_cube);
    } else if (kind == ElasticityKind::IsotropicARAP) {
      compute_hessian<2, elasticity::IsotropicARAP>
          <<<n_block, n_thread>>>(deform_grad, lame, svd_u, svd_v, svd_s,
                                  energy_density, pk1, local_hessian, n_cube);
    } else {
      AX_THROW_RUNTIME_ERROR("Unsupported elasticity model");
    }
  } else {
    if (kind == ElasticityKind::Linear) {
      compute_hessian<3, elasticity::Linear>
          <<<n_block, n_thread>>>(deform_grad, lame, svd_u, svd_v, svd_s,
                                  energy_density, pk1, local_hessian, n_cube);
    } else if (kind == ElasticityKind::NeoHookean) {
      compute_hessian<3, elasticity::NeoHookeanBW>
          <<<n_block, n_thread>>>(deform_grad, lame, svd_u, svd_v, svd_s,
                                  energy_density, pk1, local_hessian, n_cube);
    } else if (kind == ElasticityKind::StVK) {
      compute_hessian<3, elasticity::StVK>
          <<<n_block, n_thread>>>(deform_grad, lame, svd_u, svd_v, svd_s,
                                  energy_density, pk1, local_hessian, n_cube);
    } else if (kind == ElasticityKind::StableNeoHookean) {
      compute_hessian<3, elasticity::StableNeoHookean>
          <<<n_block, n_thread>>>(deform_grad, lame, svd_u, svd_v, svd_s,
                                  energy_density, pk1, local_hessian, n_cube);
    } else if (kind == ElasticityKind::IsotropicARAP) {
      compute_hessian<3, elasticity::IsotropicARAP>
          <<<n_block, n_thread>>>(deform_grad, lame, svd_u, svd_v, svd_s,
                                  energy_density, pk1, local_hessian, n_cube);
    } else {
      AX_THROW_RUNTIME_ERROR("Unsupported elasticity model");
    }
  }
}

template <int dim>
__global__ void update_svd_impl(ConstRealBufferView deform_grad,
                                RealBufferView svd_u, RealBufferView svd_v,
                                RealBufferView svd_s, size_t n_cube) {
  size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_cube)
    return;

  using MapT = math::Map<math::RealMatrix<dim, dim>>;
  using VMapT = math::Map<math::RealVector<dim>>;
  math::Map<const math::RealMatrix<dim, dim>> dg(deform_grad.Offset(0, 0, i));
  MapT u(svd_u.Offset(0, 0, i));
  MapT v(svd_v.Offset(0, 0, i));
  VMapT s(svd_s.Offset(0, i));
  math::FloatMatrix<dim, dim> U, V, Ff;
  Ff = dg.template cast<f32>();
  math::FloatVector<dim> S;
  math::decomp::svd(Ff, U, V, S);
  u = U.template cast<Real>();
  v = V.template cast<Real>();
  s = S.template cast<Real>();
}

void do_update_svd_gpu(ConstRealBufferView deform_grad, RealBufferView svd_u,
                       RealBufferView svd_v, RealBufferView svd_s) {
  size_t dim = deform_grad.Shape().X();
  size_t n_cube = deform_grad.Shape().Z();

  unsigned int n_thread = BLOCK_SIZE;
  unsigned int n_block = up_div(static_cast<unsigned int>(n_cube), n_thread);

  if (dim == 2) {
    update_svd_impl<2>
        <<<n_block, n_thread>>>(deform_grad, svd_u, svd_v, svd_s, n_cube);
  } else if (dim == 3) {
    update_svd_impl<3>
        <<<n_block, n_thread>>>(deform_grad, svd_u, svd_v, svd_s, n_cube);
  } else {
    AX_THROW_RUNTIME_ERROR("Unsupported dimension");
  }
}

} // namespace ax::fem::details