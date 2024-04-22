#include <cuda_runtime_api.h>
#include <cuda.h>
#include <thrust/copy.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/sequence.h>
#include <thrust/transform.h>
#include <thrust/zip_function.h>


#include "ax/fem/elasticity/linear.hpp"
#include "ax/fem/elasticity/neohookean_bw.hpp"
#include "ax/fem/elasticity/stable_neohookean.hpp"
#include "ax/fem/elasticity/stvk.hpp"
#include "ax/fem/elasticity_gpu.cuh"

namespace ax::fem {

template <idx dim> using SvdR = math::decomp::SvdResultImpl<dim, real>;

template <idx dim, template <idx> class ElasticModelTemplate>
struct ElasticityCompute_GPU<dim, ElasticModelTemplate>::Impl {
  thrust::device_vector<idx> seq_;
  thrust::device_vector<math::veci<dim + 1>> elements_;
  thrust::device_vector<math::matr<dim, dim>> deformation_gradient_;
  thrust::device_vector<math::matr<dim, dim>> rinv_gpu_;
  thrust::device_vector<real> rest_volume_gpu_;
  thrust::device_vector<SvdR<dim>> svd_results_;
};

template <idx dim>
__global__ void ComputeDeformationGradient(math::veci<dim + 1> const* elements,
                                           math::vecr<dim> const* pose,
                                           math::matr<dim, dim>* deformation_gradient,
                                           math::matr<dim, dim>* rinv, idx n_elem) {
  idx eid = blockIdx.x * blockDim.x + threadIdx.x;
  if (eid >= n_elem) return;

  math::veci<dim + 1> elem = elements[eid];
  math::vecr<dim> x0 = pose[elem[0]];
  math::matr<dim, dim> Dm;
  for (idx i = 0; i < dim; ++i) {
    Dm.col(i) = pose[elem[i + 1]] - x0;
  }

  deformation_gradient[eid] = Dm * rinv[eid];
}

template <idx dim>
__global__ void ComputeRestPose(math::veci<dim + 1> const* elements, math::vecr<dim> const* pose,
                                math::matr<dim, dim>* rinv, real* rest_volume, idx n_elem) {
  idx eid = blockIdx.x * blockDim.x + threadIdx.x;
  if (eid >= n_elem) return;

  math::veci<dim + 1> elem = elements[eid];
  math::vecr<dim> x0 = pose[elem[0]];
  math::matr<dim, dim> Dm;
#pragma unroll
  for (idx i = 0; i < dim; ++i) {
    Dm.col(i) = pose[elem[i + 1]] - x0;
  }

  rinv[eid] = Dm.inverse();
  real coef = dim == 3 ? 1.0 / 6.0 : 1.0 / 2.0;
  rest_volume[eid] = coef / abs(math::det(rinv[eid]));
}

template <idx dim, template <idx> class ElasticModelTemplate>
ElasticityCompute_GPU<dim, ElasticModelTemplate>::ElasticityCompute_GPU(TriMesh<dim> const& mesh)
    : ElasticityComputeBase<dim>(mesh) {
    cudaError_t error = cudaSetDevice(0);
    if (error != cudaSuccess) {
        std::cerr << "Error: " << cudaGetErrorString(error) << std::endl;
    }
    impl_ = std::make_unique<Impl>();
}

template <idx dim, template <idx> class ElasticModelTemplate>
ElasticityCompute_GPU<dim, ElasticModelTemplate>::~ElasticityCompute_GPU() {
  this->impl_.reset();
}

template <idx dim, template <idx> class ElasticModelTemplate>
bool ElasticityCompute_GPU<dim, ElasticModelTemplate>::UpdateDeformationGradient(
    math::fieldr<dim> const& pose, DeformationGradientUpdate) {
  thrust::host_vector<math::vecr<dim>> pose_cpu(pose.cols());
  for (idx i = 0; i < pose.cols(); ++i) {
    pose_cpu[i] = pose.col(i);
  }
  thrust::device_vector<math::vecr<dim>> pose_gpu(pose_cpu);
  idx n_elem = this->mesh_.GetNumElements();
  ComputeDeformationGradient<dim><<<(n_elem + 127) / 128, 128>>>(
      thrust::raw_pointer_cast(impl_->elements_.data()), thrust::raw_pointer_cast(pose_gpu.data()),
      thrust::raw_pointer_cast(impl_->deformation_gradient_.data()),
      thrust::raw_pointer_cast(impl_->rinv_gpu_.data()), n_elem);
  cudaDeviceSynchronize();
  return true;
}

template <idx dim, template <idx> class ElasticModelTemplate>
void ElasticityCompute_GPU<dim, ElasticModelTemplate>::RecomputeRestPose() {
  idx n_elem = this->mesh_.GetNumElements();
  impl_->seq_.resize(n_elem);
  thrust::sequence(impl_->seq_.begin(), impl_->seq_.end());

  // Elements
  impl_->elements_.resize(n_elem);
  thrust::host_vector<math::veci<dim + 1>> elements_host(n_elem);
  for (idx eid = 0; eid < n_elem; ++eid) {
    for (idx i = 0; i < dim + 1; ++i) {
      elements_host[eid][i] = this->mesh_.GetElement(eid)[i];
    }
  }
  thrust::copy(elements_host.begin(), elements_host.end(), impl_->elements_.begin());

  impl_->rest_volume_gpu_.resize(n_elem);
  impl_->rinv_gpu_.resize(n_elem);
  impl_->deformation_gradient_.resize(n_elem);
  impl_->svd_results_.resize(n_elem);

  auto const& pose = this->mesh_.GetVertices();
  thrust::host_vector<math::vecr<dim>> pose_cpu(pose.cols());
  for (idx i = 0; i < pose.cols(); ++i) {
    pose_cpu[i] = pose.col(i);
  }
  thrust::device_vector<math::vecr<dim>> pose_gpu(pose_cpu);
  ComputeRestPose<dim><<<(n_elem + 127) / 128, 128>>>(
      thrust::raw_pointer_cast(impl_->elements_.data()), thrust::raw_pointer_cast(pose_gpu.data()),
      thrust::raw_pointer_cast(impl_->rinv_gpu_.data()),
      thrust::raw_pointer_cast(impl_->rest_volume_gpu_.data()), n_elem);

  auto& rv_cpu= this->rest_volume_;
  auto& rinv_cpu = this->rinv_;
  rv_cpu.resize(1, n_elem);
  rinv_cpu.resize(n_elem);
  thrust::copy(impl_->rest_volume_gpu_.begin(), impl_->rest_volume_gpu_.end(), rv_cpu.data());
  thrust::copy(impl_->rinv_gpu_.begin(), impl_->rinv_gpu_.end(), rinv_cpu.data());
  auto err = cudaDeviceSynchronize();
  if (err != cudaSuccess) {
    AX_LOG(ERROR) << "Error: " << cudaGetErrorString(err);
  }
}

template <idx dim, template <idx> class ElasticModelTemplate>
math::field1r ElasticityCompute_GPU<dim, ElasticModelTemplate>::Energy(math::vec2r const& lame) {
  thrust::device_vector<real> energy_device(impl_->deformation_gradient_.size());
  thrust::transform(
      thrust::make_zip_iterator(thrust::make_tuple(impl_->deformation_gradient_.begin(),
                                                   impl_->rest_volume_gpu_.begin(),
                                                   impl_->svd_results_.begin())),
      thrust::make_zip_iterator(thrust::make_tuple(impl_->deformation_gradient_.end(), 
                                                   impl_->rest_volume_gpu_.end(),
                                                   impl_->svd_results_.end())),
      energy_device.begin(),
      thrust::make_zip_function([lame] __device__(math::matr<dim, dim> const& F,
                                                  real const& rest_volume,
                                                  SvdR<dim> const& svd) {
        return ElasticModel(lame[0], lame[1]).Energy(F, svd) * rest_volume;
      }));

  math::field1r energy(impl_->deformation_gradient_.size());
  thrust::copy(energy_device.begin(), energy_device.end(), energy.data());
  auto err = cudaDeviceSynchronize();
  if (err != cudaSuccess) {
    AX_LOG(ERROR) << "Error: " << cudaGetErrorString(err);
  }
  return energy;
};

template <idx dim, template <idx> class ElasticModelTemplate>
math::field1r ElasticityCompute_GPU<dim, ElasticModelTemplate>::Energy(math::field2r const& lame) {
  thrust::device_vector<real> energy_device(impl_->deformation_gradient_.size());
  thrust::device_vector<math::vec2r> lame_device(lame.cols());
  thrust::host_vector<math::vec2r> lame_host(lame.cols());
  for (idx i = 0; i < lame.cols(); ++i) {
    lame_host[i] = lame.col(i);
  }
  thrust::copy(lame_host.begin(), lame_host.end(), lame_device.begin());
  thrust::transform(
      thrust::make_zip_iterator(thrust::make_tuple(lame_device.begin(),
                                                   impl_->deformation_gradient_.begin(),
                                                   impl_->rest_volume_gpu_.begin(),
                                                   impl_->svd_results_.begin())),
      thrust::make_zip_iterator(thrust::make_tuple(lame_device.end(),
                                                   impl_->deformation_gradient_.end(), 
                                                   impl_->rest_volume_gpu_.end(),
                                                   impl_->svd_results_.end())),
      energy_device.begin(),
      thrust::make_zip_function([] __device__(
          math::vec2r const& lame,
          math::matr<dim, dim> const& F,
          real const& rest_volume,
          SvdR<dim> const& svd) {
        return ElasticModel(lame[0], lame[1]).Energy(F, svd) * rest_volume;
      }));

  math::field1r energy(impl_->deformation_gradient_.size());
  thrust::copy(energy_device.begin(), energy_device.end(), energy.data());
  auto err = cudaDeviceSynchronize();
  if (err != cudaSuccess) {
    AX_LOG(ERROR) << "Error: " << cudaGetErrorString(err);
  }
  return energy;
};

template <idx dim, template <idx> class ElasticModelTemplate>
List<elasticity::StressTensor<dim>> ElasticityCompute_GPU<dim, ElasticModelTemplate>::Stress(
    math::vec2r const& lame) {
  thrust::device_vector<elasticity::StressTensor<dim>> stress_device(impl_->deformation_gradient_.size());
  thrust::transform(
      thrust::make_zip_iterator(thrust::make_tuple(impl_->deformation_gradient_.begin(),
                                                   impl_->rest_volume_gpu_.begin(),
                                                   impl_->svd_results_.begin())),
      thrust::make_zip_iterator(thrust::make_tuple(impl_->deformation_gradient_.end(),
                                                   impl_->rest_volume_gpu_.end(),
                                                   impl_->svd_results_.end())),
      stress_device.begin(),
      thrust::make_zip_function([lame] __device__(math::matr<dim, dim> const& F,
                                                  real const& rest_volume,
                                                  SvdR<dim> const& svd) {
        return ElasticModel(lame[0], lame[1]).Stress(F, svd) * rest_volume;
      }));

  List<elasticity::StressTensor<dim>> stress(impl_->deformation_gradient_.size());
  thrust::copy(stress_device.begin(), stress_device.end(), stress.data());
  auto err = cudaDeviceSynchronize();
  if (err != cudaSuccess) {
    AX_LOG(ERROR) << "Error: " << cudaGetErrorString(err);
  }
  return stress;
};

template <idx dim, template <idx> class ElasticModelTemplate>
List<elasticity::StressTensor<dim>> ElasticityCompute_GPU<dim, ElasticModelTemplate>::Stress(
    math::field2r const& lame) {
  thrust::device_vector<elasticity::StressTensor<dim>> stress_device(impl_->deformation_gradient_.size());
  thrust::device_vector<math::vec2r> lame_device(lame.cols());
  thrust::host_vector<math::vec2r> lame_host(lame.cols());
  for (idx i = 0; i < lame.cols(); ++i) {
    lame_host[i] = lame.col(i);
  }
  thrust::copy(lame_host.begin(), lame_host.end(), lame_device.begin());
  thrust::transform(
      thrust::make_zip_iterator(thrust::make_tuple(lame_device.begin(),
                                                   impl_->deformation_gradient_.begin(),
                                                   impl_->rest_volume_gpu_.begin(),
                                                   impl_->svd_results_.begin())),
      thrust::make_zip_iterator(thrust::make_tuple(lame_device.end(),
                                                   impl_->deformation_gradient_.end(),
                                                   impl_->rest_volume_gpu_.end(),
                                                   impl_->svd_results_.end())),
      stress_device.begin(),
      thrust::make_zip_function([] __device__(
          math::vec2r const& lame,
          math::matr<dim, dim> const& F,
          real const& rest_volume,
          SvdR<dim> const& svd) {
        return ElasticModel(lame[0], lame[1]).Stress(F, svd) * rest_volume;
      }));

  List<elasticity::StressTensor<dim>> stress(impl_->deformation_gradient_.size());
  thrust::copy(stress_device.begin(), stress_device.end(), stress.data());
  auto err = cudaDeviceSynchronize();
  if (err != cudaSuccess) {
    AX_LOG(ERROR) << "Error: " << cudaGetErrorString(err);
  }
  return stress;
}

template <idx dim, template <idx> class ElasticModelTemplate>
List<elasticity::HessianTensor<dim>> ElasticityCompute_GPU<dim, ElasticModelTemplate>::Hessian(
    math::field2r const& lame) {
  thrust::device_vector<elasticity::HessianTensor<dim>> hessian_device(impl_->deformation_gradient_.size());
  thrust::device_vector<math::vec2r> lame_device(lame.cols());
  thrust::host_vector<math::vec2r> lame_host(lame.cols());
  for (idx i = 0; i < lame.cols(); ++i) {
    lame_host[i] = lame.col(i);
  }
  thrust::copy(lame_host.begin(), lame_host.end(), lame_device.begin());
  thrust::transform(
      thrust::make_zip_iterator(thrust::make_tuple(lame_device.begin(),
                                                   impl_->deformation_gradient_.begin(),
                                                   impl_->rest_volume_gpu_.begin(),
                                                   impl_->svd_results_.begin())),
      thrust::make_zip_iterator(thrust::make_tuple(lame_device.end(),
                                                   impl_->deformation_gradient_.end(),
                                                   impl_->rest_volume_gpu_.end(),
                                                   impl_->svd_results_.end())),
      hessian_device.begin(),
      thrust::make_zip_function([] __device__(
          math::vec2r const& lame,
          math::matr<dim, dim> const& F,
          real const& rest_volume,
          SvdR<dim> const& svd) {
        return ElasticModel(lame[0], lame[1]).Hessian(F, svd) * rest_volume;
      }));

  List<elasticity::HessianTensor<dim>> hessian(impl_->deformation_gradient_.size());
  thrust::copy(hessian_device.begin(), hessian_device.end(), hessian.data());
  cudaDeviceSynchronize();
  return hessian;
}

template <idx dim, template <idx> class ElasticModelTemplate>
List<elasticity::HessianTensor<dim>> ElasticityCompute_GPU<dim, ElasticModelTemplate>::Hessian(
    math::vec2r const& lame) {
  thrust::device_vector<elasticity::HessianTensor<dim>> hessian_device(impl_->deformation_gradient_.size());
  thrust::transform(
      thrust::make_zip_iterator(thrust::make_tuple(impl_->deformation_gradient_.begin(),
                                                   impl_->rest_volume_gpu_.begin(),
                                                   impl_->svd_results_.begin())),
      thrust::make_zip_iterator(thrust::make_tuple(impl_->deformation_gradient_.end(),
                                                   impl_->rest_volume_gpu_.end(),
                                                   impl_->svd_results_.end())),
      hessian_device.begin(),
      thrust::make_zip_function([lame] __device__(math::matr<dim, dim> const& F,
                                                  real const& rest_volume,
                                                  SvdR<dim> const& svd) {
        return ElasticModel(lame[0], lame[1]).Hessian(F, svd) * rest_volume;
      }));

  List<elasticity::HessianTensor<dim>> hessian(impl_->deformation_gradient_.size());
  thrust::copy(hessian_device.begin(), hessian_device.end(), hessian.data());
  cudaDeviceSynchronize();
  return hessian;
}

// NOTE: Currently, ARAP relies on Jacobi SVD.
// template class ElasticityCompute_GPU<2, elasticity::StableNeoHookean>;
// template class ElasticityCompute_GPU<2, elasticity::NeoHookeanBW>;
// template class ElasticityCompute_GPU<2, elasticity::StVK>;
// template class ElasticityCompute_GPU<2, elasticity::Linear>;
// template class ElasticityCompute_GPU<2, elasticity::IsotropicARAP>;

template class ElasticityCompute_GPU<3, elasticity::StableNeoHookean>;
// template class ElasticityCompute_GPU<3, elasticity::NeoHookeanBW>;
// template class ElasticityCompute_GPU<3, elasticity::StVK>;
// template class ElasticityCompute_GPU<3, elasticity::Linear>;
// template class ElasticityCompute_GPU<3, elasticity::IsotropicARAP>;

}  // namespace ax::fem
