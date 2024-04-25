#include "ax/fem/elasticity/arap.hpp"
#include "ax/math/decomp/svd/remove_rotation.hpp"
#include "ax/math/decomp/svd/svd_cuda.cuh"
#ifndef AX_HAS_CUDA
#  error "This file should only be included in CUDA mode"
#endif

#include <cuda.h>
#include <cuda_runtime_api.h>
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
#include "ax/optim/spsdm/eigenvalue.hpp"

namespace ax::fem {

class Timed {
public:
  Timed(const char* name) : start(absl::GetCurrentTimeNanos()), name(name) {}

  ~Timed() {
    end = absl::GetCurrentTimeNanos();
    AX_LOG(INFO) << "Timed [" << name << "] done in" << (end - start) * 1e-6 << "ms.";
  }

  int64_t start, end;
  const char* name;
};
#ifdef ENABLE_TIMER
#define TimeThisOne() Timed timer_do_not_use(__PRETTY_FUNCTION__)
#else
#define TimeThisOne()
#endif

template <idx dim> using SvdR = math::decomp::SvdResultImpl<dim, real>;

struct VertexOnElementInfo {
  idx vert_id;
  idx elem_id;
  idx local_id;
  AX_HOST_DEVICE VertexOnElementInfo(idx v, idx e, idx l) : vert_id(v), elem_id(e), local_id(l) {}

  AX_HOST_DEVICE VertexOnElementInfo(const VertexOnElementInfo& other) = default;
};

struct COO {
  idx row;
  idx col;
  real val;
};

template <idx dim, template <idx> class ElasticModelTemplate>
struct ElasticityCompute_GPU<dim, ElasticModelTemplate>::Impl {
  thrust::device_vector<math::veci<dim + 1>> elements_;
  thrust::device_vector<math::vecr<dim>> pose_gpu_;  ///< current pose
  thrust::device_vector<math::matr<dim, dim>> deformation_gradient_;
  thrust::device_vector<math::matr<dim, dim>> rinv_gpu_;
  thrust::device_vector<real> rest_volume_gpu_;
  thrust::device_vector<SvdR<dim>> svd_results_;

  thrust::device_vector<idx> gather_entries_;
  thrust::device_vector<VertexOnElementInfo> gather_information_;

  thrust::device_vector<math::matr<dim, dim>> stress_on_elements_;
  thrust::device_vector<math::vecr<dim>> stress_on_vertices_;
  thrust::device_vector<math::matr<dim * dim, dim * dim>> hessian_on_elements_;
  thrust::device_vector<COO> hessian_on_vertices_;
  thrust::device_vector<real> energy_on_vertices_;
  thrust::device_vector<real> energy_on_elements_;
  thrust::device_vector<math::vec2r> lame_;
};

#define GPU_GRAIN 128

template <idx dim>
__global__ void compute_deformation_gradient(math::veci<dim + 1> const* elements,
                                             math::vecr<dim> const* pose,
                                             math::matr<dim, dim>* deformation_gradient,
                                             math::matr<dim, dim>* rinv, SvdR<dim>* svdr,
                                             idx n_elem, bool need_svd) {
  idx eid = blockIdx.x * blockDim.x + threadIdx.x;
  if (eid >= n_elem) return;

  const math::veci<dim + 1> elem = elements[eid];
  const math::vecr<dim> x0 = pose[elem[0]];
  math::matr<dim, dim> Dm;
#pragma unroll
  for (idx i = 0; i < dim; ++i) {
    Dm.col(i) = pose[elem[i + 1]] - x0;
  }

  math::matr<dim, dim> F = Dm * rinv[eid];
  deformation_gradient[eid] = F;
  if (need_svd) {
    math::mat3f U, V, Ff;
    Ff = F.template cast<float>();
    math::vec3f S;
    math::decomp::svd(Ff, U, V, S);
    svdr[eid].U_ = U.cast<real>();
    svdr[eid].V_ = V.cast<real>();
    svdr[eid].sigma_ = S.cast<real>();
    ax::math::decomp::svd_remove_rotation<ax::real>(svdr[eid]);
  }
}

template <idx dim>
__global__ void compute_rest_pose(math::veci<dim + 1> const* elements, math::vecr<dim> const* pose,
                                  math::matr<dim, dim>* rinv, real* rest_volume, idx n_elem) {
  idx eid = blockIdx.x * blockDim.x + threadIdx.x;
  if (eid >= n_elem) return;

  const math::veci<dim + 1> elem = elements[eid];
  const math::vecr<dim> x0 = pose[elem[0]];
  math::matr<dim, dim> Dm;
#pragma unroll
  for (idx i = 0; i < dim; ++i) {
    Dm.col(i) = pose[elem[i + 1]] - x0;
  }

  rinv[eid] = Dm.inverse();
  const real coef = dim == 3 ? 1.0 / 6.0 : 1.0 / 2.0;
  rest_volume[eid] = coef / abs(math::det(rinv[eid]));

  // printf("RestVolume of %ld is %lf\n", eid, rest_volume[eid]);
}

template <idx dim, template <idx> class ElasticModelTemplate>
ElasticityCompute_GPU<dim, ElasticModelTemplate>::ElasticityCompute_GPU(MeshPtr const& mesh)
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
bool ElasticityCompute_GPU<dim, ElasticModelTemplate>::Update(math::fieldr<dim> const& pose,
                                                              ElasticityUpdateLevel upt) {
  TimeThisOne();
  auto error = cudaMemcpy(thrust::raw_pointer_cast(impl_->pose_gpu_.data()), pose.data(),
                          pose.size() * sizeof(real), cudaMemcpyHostToDevice);
  AX_CHECK(error == cudaSuccess) << "Failed to copy pose to GPU." << cudaGetErrorString(error);
  idx n_elem = this->mesh_->GetNumElements();
  bool const need_svd
      = (ElasticModel{}.EnergyRequiresSvd() && upt == ElasticityUpdateLevel::kEnergy)
        || (ElasticModel{}.StressRequiresSvd() && upt == ElasticityUpdateLevel::kStress)
        || (ElasticModel{}.HessianRequiresSvd() && upt == ElasticityUpdateLevel::kHessian);
  compute_deformation_gradient<dim><<<(n_elem + GPU_GRAIN - 1) / GPU_GRAIN, GPU_GRAIN>>>(
      thrust::raw_pointer_cast(impl_->elements_.data()),
      thrust::raw_pointer_cast(impl_->pose_gpu_.data()),
      thrust::raw_pointer_cast(impl_->deformation_gradient_.data()),
      thrust::raw_pointer_cast(impl_->rinv_gpu_.data()),
      thrust::raw_pointer_cast(impl_->svd_results_.data()), n_elem, need_svd);

  // thrust::host_vector<math::matr<dim, dim>> deformation_gradient_cpu(n_elem);
  // thrust::copy(impl_->deformation_gradient_.begin(), impl_->deformation_gradient_.end(),
  //              deformation_gradient_cpu.begin());
  // std::cout << "Deformation gradient: " << deformation_gradient_cpu[0] << std::endl;
  return true;
}

template <idx dim, template <idx> class ElasticModelTemplate>
void ElasticityCompute_GPU<dim, ElasticModelTemplate>::RecomputeRestPose() {
  TimeThisOne();
  auto const& mesh = *(this->mesh_);
  idx n_elem = this->mesh_->GetNumElements();
  idx n_vert = this->mesh_->GetNumVertices();

  // Elements
  impl_->elements_.resize(n_elem);
  thrust::host_vector<math::veci<dim + 1>> elements_host(n_elem);
  for (idx eid = 0; eid < n_elem; ++eid) {
    auto elem = mesh.GetElement(eid);
    for (idx i = 0; i < dim + 1; ++i) {
      elements_host[eid][i] = elem[i];
    }
  }
  thrust::copy(elements_host.begin(), elements_host.end(), impl_->elements_.begin());

  impl_->rest_volume_gpu_.resize(n_elem);
  impl_->rinv_gpu_.resize(n_elem);
  impl_->deformation_gradient_.resize(n_elem);
  impl_->svd_results_.resize(n_elem);

  auto const& pose = mesh.GetVertices();
  thrust::host_vector<math::vecr<dim>> pose_cpu(pose.cols());
  for (idx i = 0; i < pose.cols(); ++i) {
    pose_cpu[i] = pose.col(i);
  }
  impl_->pose_gpu_ = pose_cpu;
  compute_rest_pose<dim><<<(n_elem + GPU_GRAIN - 1) / GPU_GRAIN, GPU_GRAIN>>>(
      thrust::raw_pointer_cast(impl_->elements_.data()),
      thrust::raw_pointer_cast(impl_->pose_gpu_.data()),
      thrust::raw_pointer_cast(impl_->rinv_gpu_.data()),
      thrust::raw_pointer_cast(impl_->rest_volume_gpu_.data()), n_elem);

  auto& rv_cpu = this->rest_volume_;
  auto& rinv_cpu = this->rinv_;
  rv_cpu.resize(1, n_elem);
  rinv_cpu.resize(n_elem);
  thrust::copy(impl_->rest_volume_gpu_.begin(), impl_->rest_volume_gpu_.end(), rv_cpu.data());
  thrust::copy(impl_->rinv_gpu_.begin(), impl_->rinv_gpu_.end(), rinv_cpu.data());

  impl_->energy_on_vertices_.resize(n_vert);
  impl_->energy_on_elements_.resize(n_elem);
  impl_->stress_on_vertices_.resize(n_vert);
  impl_->stress_on_elements_.resize(n_elem);
  impl_->hessian_on_vertices_.resize(n_elem * (dim + 1) * dim * (dim + 1) * dim);
  impl_->hessian_on_elements_.resize(n_elem);
  impl_->lame_.resize(n_elem);

  thrust::host_vector<idx> gather_entries_cpu;
  thrust::host_vector<VertexOnElementInfo> info;
  for (idx e = 0; e < n_elem; ++e) {
    auto const& elem = mesh.GetElement(e);
    for (idx d = 0; d <= dim; ++d) {
      info.push_back(VertexOnElementInfo{elem[d], e, d});
    }
  }

  std::sort(info.begin(), info.end(),
            [](VertexOnElementInfo const& a, VertexOnElementInfo const& b) {
              return a.vert_id < b.vert_id;
            });

  gather_entries_cpu.resize(n_vert, -1);
  for (idx i = 0; (size_t)i < info.size(); ++i) {
    auto v = info[i].vert_id;
    if (gather_entries_cpu[v] == -1) {
      gather_entries_cpu[v] = i;
      // printf("V%ld -> %ld\n", v, i);
    }
  }

  for (idx i = 0; (size_t)i < info.size(); ++i) {
    // AX_CHECK(info[i].vert_id < n_vert && info[i].vert_id >= 0) << i;
    // AX_CHECK(info[i].elem_id < n_elem && info[i].elem_id >= 0) << i;
    // AX_CHECK(info[i].local_id <= dim && info[i].local_id >= 0) << i;
    // printf("V%ld E%ld L%ld\n", info[i].vert_id, info[i].elem_id, info[i].local_id);
  }

  impl_->gather_entries_.resize(n_vert);
  thrust::copy(gather_entries_cpu.begin(), gather_entries_cpu.end(),
               impl_->gather_entries_.begin());
  impl_->gather_information_ = info;

  ElasticityComputeBase<dim>::RecomputeRestPose();
}

template <idx dim, template <idx> class ElasticModelTemplate>
void ElasticityCompute_GPU<dim, ElasticModelTemplate>::SetLame(math::field2r const& lame) {
  thrust::host_vector<math::vec2r> lame_host(lame.cols());
  for (idx i = 0; i < lame.cols(); ++i) {
    lame_host[i] = lame.col(i);
  }
  impl_->lame_ = lame_host;
}

template <idx dim, template <idx> class ElasticModelTemplate>
void ElasticityCompute_GPU<dim, ElasticModelTemplate>::SetLame(math::vec2r const& lame) {
  thrust::host_vector<math::vec2r> lame_host(this->mesh_->GetNumElements(), lame);
  impl_->lame_ = lame_host;
}

/*************************
 * SECT: Energy
 *************************/

template <idx dim, template <idx> class ElasticModelTemplate>
__global__ void compute_energy_impl(math::matr<dim, dim>* deformation_gradient, real* rest_volume,
                                    SvdR<dim>* svd_results, real* energy, math::vec2r* lame,
                                    idx n_elem) {
  idx eid = blockIdx.x * blockDim.x + threadIdx.x;
  if (eid >= n_elem) return;
  energy[eid] = ElasticModelTemplate<dim>(lame[eid][0], lame[eid][1])
                    .Energy(deformation_gradient[eid], svd_results[eid])
                * rest_volume[eid];
}

template <idx dim, template <idx> class ElasticModelTemplate>
__global__ void compute_energy_impl(math::matr<dim, dim>* deformation_gradient, real* rest_volume,
                                    SvdR<dim>* svd_results, real* energy, real lambda, real mu,
                                    idx n_elem) {
  idx eid = blockIdx.x * blockDim.x + threadIdx.x;
  if (eid >= n_elem) return;
  energy[eid]
      = ElasticModelTemplate<dim>(lambda, mu).Energy(deformation_gradient[eid], svd_results[eid])
        * rest_volume[eid];
}

template <idx dim, template <idx> class ElasticModelTemplate>
math::field1r ElasticityCompute_GPU<dim, ElasticModelTemplate>::Energy(math::vec2r const& lame) {
  TimeThisOne();
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
                                                  real const& rest_volume, SvdR<dim> const& svd) {
        return ElasticModel(lame[0], lame[1]).Energy(F, svd) * rest_volume;
      }));
  // compute_energy_impl<dim, ElasticModelTemplate><<<(impl_->deformation_gradient_.size() + GPU_GRAIN - 1) /
  // GPU_GRAIN, GPU_GRAIN>>>(
  //     thrust::raw_pointer_cast(impl_->deformation_gradient_.data()),
  //     thrust::raw_pointer_cast(impl_->rest_volume_gpu_.data()),
  //     thrust::raw_pointer_cast(impl_->svd_results_.data()),
  //     thrust::raw_pointer_cast(energy_device.data()),
  //     lame[0], lame[1],
  //     impl_->deformation_gradient_.size());
  math::field1r energy(impl_->deformation_gradient_.size());
  thrust::copy(energy_device.begin(), energy_device.end(), energy.data());
  return energy;
};

template <idx dim, template <idx> class ElasticModelTemplate>
math::field1r ElasticityCompute_GPU<dim, ElasticModelTemplate>::Energy(math::field2r const& lame) {
  TimeThisOne();
  thrust::device_vector<real> energy_device(impl_->deformation_gradient_.size());
  thrust::device_vector<math::vec2r> lame_device(lame.cols());
  thrust::host_vector<math::vec2r> lame_host(lame.cols());
  for (idx i = 0; i < lame.cols(); ++i) {
    lame_host[i] = lame.col(i);
  }
  thrust::copy(lame_host.begin(), lame_host.end(), lame_device.begin());
  thrust::transform(thrust::make_zip_iterator(thrust::make_tuple(
                        lame_device.begin(), impl_->deformation_gradient_.begin(),
                        impl_->rest_volume_gpu_.begin(), impl_->svd_results_.begin())),
                    thrust::make_zip_iterator(thrust::make_tuple(
                        lame_device.end(), impl_->deformation_gradient_.end(),
                        impl_->rest_volume_gpu_.end(), impl_->svd_results_.end())),
                    energy_device.begin(),
                    thrust::make_zip_function(
                        [] __device__(math::vec2r const& lame, math::matr<dim, dim> const& F,
                                      real const& rest_volume, SvdR<dim> const& svd) {
                          return ElasticModel(lame[0], lame[1]).Energy(F, svd) * rest_volume;
                        }));

  // NOTE: Naive CUDA:
  // compute_energy_impl<dim, ElasticModelTemplate><<<(impl_->deformation_gradient_.size() + GPU_GRAIN - 1) /
  // GPU_GRAIN, GPU_GRAIN>>>(
  //     thrust::raw_pointer_cast(impl_->deformation_gradient_.data()),
  //     thrust::raw_pointer_cast(impl_->rest_volume_gpu_.data()),
  //     thrust::raw_pointer_cast(impl_->svd_results_.data()),
  //     thrust::raw_pointer_cast(energy_device.data()),
  //     thrust::raw_pointer_cast(lame_device.data()),
  //     impl_->deformation_gradient_.size());
  math::field1r energy(impl_->deformation_gradient_.size());
  cudaMemcpy(energy.data(), thrust::raw_pointer_cast(energy_device.data()),
             energy.size() * sizeof(real), cudaMemcpyDeviceToHost);
  return energy;
};

/*************************
 * SECT: Stress
 *************************/
template <idx dim, template <idx> class ElasticModelTemplate>
__global__ void compute_stress_impl(math::matr<dim, dim>* deformation_gradient, real* rest_volume,
                                    SvdR<dim>* svd_results, elasticity::StressTensor<dim>* stress,
                                    real lambda, real mu, idx n_elem) {
  size_t eid = blockIdx.x * blockDim.x + threadIdx.x;
  if (eid >= n_elem) return;

  stress[eid]
      = ElasticModelTemplate<dim>(lambda, mu).Stress(deformation_gradient[eid], svd_results[eid])
        * rest_volume[eid];
}

template <idx dim, template <idx> class ElasticModelTemplate>
__global__ void compute_stress_impl(math::matr<dim, dim> const* deformation_gradient,
                                    real const* rest_volume, SvdR<dim> const* svd_results,
                                    math::vec2r const* lame, elasticity::StressTensor<dim>* stress,
                                    idx n_elem) {
  size_t eid = blockIdx.x * blockDim.x + threadIdx.x;
  if (eid >= n_elem) return;
  stress[eid] = ElasticModelTemplate<dim>(lame[eid][0], lame[eid][1])
                    .Stress(deformation_gradient[eid], svd_results[eid])
                * rest_volume[eid];
}

template <idx dim, template <idx> class ElasticModelTemplate>
List<elasticity::StressTensor<dim>> ElasticityCompute_GPU<dim, ElasticModelTemplate>::Stress(
    math::vec2r const& lame) {
  thrust::device_vector<elasticity::StressTensor<dim>> stress_device(
      impl_->deformation_gradient_.size());
  thrust::transform(
      thrust::make_zip_iterator(thrust::make_tuple(impl_->deformation_gradient_.begin(),
                                                   impl_->rest_volume_gpu_.begin(),
                                                   impl_->svd_results_.begin())),
      thrust::make_zip_iterator(thrust::make_tuple(impl_->deformation_gradient_.end(),
                                                   impl_->rest_volume_gpu_.end(),
                                                   impl_->svd_results_.end())),
      stress_device.begin(),
      thrust::make_zip_function([lame] __device__(math::matr<dim, dim> const& F,
                                                  real const& rest_volume, SvdR<dim> const& svd) {
        return ElasticModel(lame[0], lame[1]).Stress(F, svd) * rest_volume;
      }));

  // compute_stress_impl<dim, ElasticModelTemplate><<<(impl_->deformation_gradient_.size() + GPU_GRAIN - 1) /
  // GPU_GRAIN, GPU_GRAIN>>>(
  //     thrust::raw_pointer_cast(impl_->deformation_gradient_.data()),
  //     thrust::raw_pointer_cast(impl_->rest_volume_gpu_.data()),
  //     thrust::raw_pointer_cast(impl_->svd_results_.data()),
  //     thrust::raw_pointer_cast(stress_device.data()),
  //     lame[0], lame[1],
  //     impl_->deformation_gradient_.size());
  List<elasticity::StressTensor<dim>> stress(impl_->deformation_gradient_.size());

  thrust::copy(stress_device.begin(), stress_device.end(), stress.data());
  return stress;
};

template <idx dim, template <idx> class ElasticModelTemplate>
List<elasticity::StressTensor<dim>> ElasticityCompute_GPU<dim, ElasticModelTemplate>::Stress(
    math::field2r const& lame) {
  thrust::device_vector<elasticity::StressTensor<dim>> stress_device(
      impl_->deformation_gradient_.size());
  thrust::device_vector<math::vec2r> lame_device(lame.cols());
  thrust::host_vector<math::vec2r> lame_host(lame.cols());
  for (idx i = 0; i < lame.cols(); ++i) {
    lame_host[i] = lame.col(i);
  }
  thrust::copy(lame_host.begin(), lame_host.end(), lame_device.begin());
  thrust::transform(thrust::make_zip_iterator(thrust::make_tuple(
                        lame_device.begin(), impl_->deformation_gradient_.begin(),
                        impl_->rest_volume_gpu_.begin(), impl_->svd_results_.begin())),
                    thrust::make_zip_iterator(thrust::make_tuple(
                        lame_device.end(), impl_->deformation_gradient_.end(),
                        impl_->rest_volume_gpu_.end(), impl_->svd_results_.end())),
                    stress_device.begin(),
                    thrust::make_zip_function(
                        [] __device__(math::vec2r const& lame, math::matr<dim, dim> const& F,
                                      real const& rest_volume, SvdR<dim> const& svd) {
                          return ElasticModel(lame[0], lame[1]).Stress(F, svd) * rest_volume;
                        }));

  List<elasticity::StressTensor<dim>> stress(impl_->deformation_gradient_.size());
  thrust::copy(stress_device.begin(), stress_device.end(), stress.data());
  return stress;
}

/*************************
 * SECT: Hessian
 *************************/

template <idx dim, template <idx> class ElasticModelTemplate>
__global__ void compute_hessian_impl(math::matr<dim, dim>* deformation_gradient, real* rest_volume,
                                     SvdR<dim>* svd_results,
                                     elasticity::HessianTensor<dim>* hessian, real lambda, real mu,
                                     idx n_elem) {
  idx eid = blockIdx.x * blockDim.x + threadIdx.x;
  if (eid >= n_elem) return;
  hessian[eid]
      = ElasticModelTemplate<dim>(lambda, mu).Hessian(deformation_gradient[eid], svd_results[eid])
        * rest_volume[eid];
}

template <idx dim, template <idx> class ElasticModelTemplate>
__global__ void compute_hessian_impl(math::matr<dim, dim>* deformation_gradient, real* rest_volume,
                                     SvdR<dim>* svd_results,
                                     elasticity::HessianTensor<dim>* hessian, math::vec2r* lame,
                                     idx n_elem) {
  idx eid = blockIdx.x * blockDim.x + threadIdx.x;
  if (eid >= n_elem) return;
  hessian[eid] = ElasticModelTemplate<dim>(lame[eid][0], lame[eid][1])
                     .Hessian(deformation_gradient[eid], svd_results[eid])
                 * rest_volume[eid];
}

template <idx dim, template <idx> class ElasticModelTemplate>
List<elasticity::HessianTensor<dim>> ElasticityCompute_GPU<dim, ElasticModelTemplate>::Hessian(
    math::field2r const& lame) {
  thrust::device_vector<elasticity::HessianTensor<dim>> hessian_device(
      impl_->deformation_gradient_.size());
  thrust::device_vector<math::vec2r> lame_device(lame.cols());
  thrust::host_vector<math::vec2r> lame_host(lame.cols());
  for (idx i = 0; i < lame.cols(); ++i) {
    lame_host[i] = lame.col(i);
  }
  thrust::copy(lame_host.begin(), lame_host.end(), lame_device.begin());
  thrust::transform(thrust::make_zip_iterator(thrust::make_tuple(
                        lame_device.begin(), impl_->deformation_gradient_.begin(),
                        impl_->rest_volume_gpu_.begin(), impl_->svd_results_.begin())),
                    thrust::make_zip_iterator(thrust::make_tuple(
                        lame_device.end(), impl_->deformation_gradient_.end(),
                        impl_->rest_volume_gpu_.end(), impl_->svd_results_.end())),
                    hessian_device.begin(),
                    thrust::make_zip_function(
                        [] __device__(math::vec2r const& lame, math::matr<dim, dim> const& F,
                                      real const& rest_volume, SvdR<dim> const& svd) {
                          return ElasticModel(lame[0], lame[1]).Hessian(F, svd) * rest_volume;
                        }));

  // compute_hessian_impl<dim, ElasticModelTemplate><<<(impl_->deformation_gradient_.size() + GPU_GRAIN - 1) /
  // GPU_GRAIN, GPU_GRAIN>>>(
  //     thrust::raw_pointer_cast(impl_->deformation_gradient_.data()),
  //     thrust::raw_pointer_cast(impl_->rest_volume_gpu_.data()),
  //     thrust::raw_pointer_cast(impl_->svd_results_.data()),
  //     thrust::raw_pointer_cast(hessian_device.data()),
  //     thrust::raw_pointer_cast(lame_device.data()),
  //     impl_->deformation_gradient_.size());
  //
  List<elasticity::HessianTensor<dim>> hessian(impl_->deformation_gradient_.size());
  thrust::copy(hessian_device.begin(), hessian_device.end(), hessian.data());
  return hessian;
}

template <idx dim, template <idx> class ElasticModelTemplate>
List<elasticity::HessianTensor<dim>> ElasticityCompute_GPU<dim, ElasticModelTemplate>::Hessian(
    math::vec2r const& lame) {
  thrust::device_vector<elasticity::HessianTensor<dim>> hessian_device(
      impl_->deformation_gradient_.size());
  thrust::transform(
      thrust::make_zip_iterator(thrust::make_tuple(impl_->deformation_gradient_.begin(),
                                                   impl_->rest_volume_gpu_.begin(),
                                                   impl_->svd_results_.begin())),
      thrust::make_zip_iterator(thrust::make_tuple(impl_->deformation_gradient_.end(),
                                                   impl_->rest_volume_gpu_.end(),
                                                   impl_->svd_results_.end())),
      hessian_device.begin(),
      thrust::make_zip_function([lame] __device__(math::matr<dim, dim> const& F,
                                                  real const& rest_volume, SvdR<dim> const& svd) {
        return ElasticModel(lame[0], lame[1]).Hessian(F, svd) * rest_volume;
      }));

  // compute_hessian_impl<dim, ElasticModelTemplate><<<(impl_->deformation_gradient_.size() + GPU_GRAIN - 1) /
  // GPU_GRAIN, GPU_GRAIN>>>(
  //     thrust::raw_pointer_cast(impl_->deformation_gradient_.data()),
  //     thrust::raw_pointer_cast(impl_->rest_volume_gpu_.data()),
  //     thrust::raw_pointer_cast(impl_->svd_results_.data()),
  //     thrust::raw_pointer_cast(hessian_device.data()),
  //     lame[0], lame[1],
  //     impl_->deformation_gradient_.size());

  List<elasticity::HessianTensor<dim>> hessian(impl_->deformation_gradient_.size());
  thrust::copy(hessian_device.begin(), hessian_device.end(), hessian.data());
  return hessian;
}

template <idx dim, template <idx> class ElasticModelTemplate>
void ElasticityCompute_GPU<dim, ElasticModelTemplate>::UpdateEnergy() {
  TimeThisOne();
  auto& energy_on_elements = impl_->energy_on_elements_;
  auto const& lame = impl_->lame_;
  auto const& F = impl_->deformation_gradient_;
  auto const& rest_volume = impl_->rest_volume_gpu_;
  auto const& svd = impl_->svd_results_;

  // thrust::transform(
  //     thrust::make_zip_iterator(F.begin(), rest_volume.begin(), lame.begin(), svd.begin()),
  //     thrust::make_zip_iterator(F.end(), rest_volume.end(), lame.end(), svd.begin()),
  //     energy_on_elements.begin(),
  //     thrust::make_zip_function([] __device__(math::matr<dim, dim> const& F, real rest_volume,
  //                                             math::vec2r lame, SvdR<dim> const& svd) {
  //       ElasticModelTemplate<dim> model(lame[0], lame[1]);
  //       return rest_volume * model.Energy(F, svd);
  //     }));
  compute_energy_impl<dim, ElasticModelTemplate>
      <<<(impl_->deformation_gradient_.size() + GPU_GRAIN - 1) / GPU_GRAIN, GPU_GRAIN>>>(
          thrust::raw_pointer_cast(impl_->deformation_gradient_.data()),
          thrust::raw_pointer_cast(impl_->rest_volume_gpu_.data()),
          thrust::raw_pointer_cast(impl_->svd_results_.data()),
          thrust::raw_pointer_cast(impl_->energy_on_elements_.data()),
          thrust::raw_pointer_cast(impl_->lame_.data()), impl_->deformation_gradient_.size());
}

template <idx dim, template <idx> class ElasticModelTemplate>
void ElasticityCompute_GPU<dim, ElasticModelTemplate>::UpdateStress() {
  TimeThisOne();
  auto& stress_on_elements = impl_->stress_on_elements_;
  auto const& lame = impl_->lame_;
  auto const& F = impl_->deformation_gradient_;
  auto const& rest_volume = impl_->rest_volume_gpu_;
  auto const& svd = impl_->svd_results_;

  // thrust::transform(
  //     thrust::make_zip_iterator(F.begin(), rest_volume.begin(), lame.begin(), svd.begin()),
  //     thrust::make_zip_iterator(F.end(), rest_volume.end(), lame.end(), svd.begin()),
  //     stress_on_elements.begin(),
  //     thrust::make_zip_function([] __device__(math::matr<dim, dim> const& F, real rest_volume,
  //                                             math::vec2r lame, SvdR<dim> const& svd) {
  //       ElasticModelTemplate<dim> model(lame[0], lame[1]);
  //       return rest_volume * model.Stress(F, svd);
  //     }));
  compute_stress_impl<dim, ElasticModelTemplate>
      <<<(impl_->deformation_gradient_.size() + GPU_GRAIN - 1) / GPU_GRAIN, GPU_GRAIN>>>(
          thrust::raw_pointer_cast(impl_->deformation_gradient_.data()),
          thrust::raw_pointer_cast(impl_->rest_volume_gpu_.data()),
          thrust::raw_pointer_cast(impl_->svd_results_.data()),
          thrust::raw_pointer_cast(impl_->lame_.data()),
          thrust::raw_pointer_cast(impl_->stress_on_elements_.data()),
          impl_->deformation_gradient_.size());
}

template <idx dim, template <idx> class ElasticModelTemplate>
void ElasticityCompute_GPU<dim, ElasticModelTemplate>::UpdateHessian(bool projection) {
  TimeThisOne();
  auto& hessian_on_elements = impl_->hessian_on_elements_;
  auto const& lame = impl_->lame_;
  auto const& F = impl_->deformation_gradient_;
  auto const& rest_volume = impl_->rest_volume_gpu_;
  auto const& svd = impl_->svd_results_;

  // thrust::transform(
  //     thrust::make_zip_iterator(F.begin(), rest_volume.begin(), lame.begin(), svd.begin()),
  //     thrust::make_zip_iterator(F.end(), rest_volume.end(), lame.end(), svd.begin()),
  //     hessian_on_elements.begin(),
  //     thrust::make_zip_function([projection] __device__(math::matr<dim, dim> const& F,
  //                                                       real rest_volume, math::vec2r lame,
  //                                                       SvdR<dim> const& svd) {
  //       ElasticModelTemplate<dim> model(lame[0], lame[1]);
  //       math::matr<dim * dim, dim * dim> H = rest_volume * model.Hessian(F, svd);
  //       if (projection) {
  //         Eigen::SelfAdjointEigenSolver<math::matr<dim * dim, dim * dim>> es(H);
  //         math::vecr<dim * dim> D = es.eigenvalues().cwiseMax(1e-4);
  //         math::matr<dim * dim, dim * dim> V = es.eigenvectors();
  //         H = V * D.asDiagonal() * V.transpose();
  //       }
  //       return H;
  //     }));
  compute_hessian_impl<dim, ElasticModelTemplate>
      <<<(impl_->deformation_gradient_.size() + GPU_GRAIN - 1) / GPU_GRAIN, GPU_GRAIN>>>(
          thrust::raw_pointer_cast(impl_->deformation_gradient_.data()),
          thrust::raw_pointer_cast(impl_->rest_volume_gpu_.data()),
          thrust::raw_pointer_cast(impl_->svd_results_.data()),
          thrust::raw_pointer_cast(impl_->hessian_on_elements_.data()),
          thrust::raw_pointer_cast(impl_->lame_.data()), impl_->deformation_gradient_.size());
}

template <idx dim> __global__ void gather_energy(real* energy_on_vertices,
                                                 real const* energy_on_elements, idx const* entry,
                                                 VertexOnElementInfo const* gather_info, idx n_vert,
                                                 idx n_info) {
  idx vid = blockIdx.x * blockDim.x + threadIdx.x;
  if (vid >= n_vert) return;
  idx e = entry[vid];
  real total_energy = 0;
  if (e == -1) {
    energy_on_vertices[vid] = 0;
    return;
  }
  for (idx i = e; i < n_info; ++i) {
    if (gather_info[i].vert_id == vid) {
      total_energy += energy_on_elements[gather_info[i].elem_id];
    } else {
      break;
    }
  }
  energy_on_vertices[vid] = total_energy / real(dim + 1);
}

template <idx dim> __global__ void gather_stress(math::vecr<dim>* stress_on_vertices,
                                                 math::matr<dim, dim> const* stress_on_elements,
                                                 math::matr<dim, dim> const* rinv, idx const* entry,
                                                 VertexOnElementInfo const* gather_info, idx n_vert,
                                                 idx n_elem, idx n_info) {
  const idx vid = blockIdx.x * blockDim.x + threadIdx.x;
  if (vid >= n_vert) return;
  idx const e = entry[vid];
  math::vecr<dim> total;
  total.setZero();
  if (e == -1) {
    stress_on_vertices[vid] = total;
    return;
  }
  for (idx i = e; i < n_info; ++i) {
    if (gather_info[i].vert_id == vid) {
      const idx eid = gather_info[i].elem_id;
      math::matr<dim, dim> const& Dm_inv = rinv[eid];
      math::matr<dim, dim> const& Pk1 = stress_on_elements[eid];
      math::matr<dim, dim> Pk = Pk1 * Dm_inv.transpose();
      if (gather_info[i].local_id == 0) {
        for (idx D = 0; D < dim; ++D) {
          total -= Pk.col(D);
        }
      } else {
        total += Pk.col(gather_info[i].local_id - 1);
      }
    } else {
      break;
    }
  }

  stress_on_vertices[vid] = total;
}

template <idx dim, template <idx> class ElasticModelTemplate>
void ElasticityCompute_GPU<dim, ElasticModelTemplate>::GatherEnergyToVertices() {
  TimeThisOne();
  auto& ev = impl_->energy_on_vertices_;
  auto const& ee = impl_->energy_on_elements_;
  auto const& ge = impl_->gather_entries_;
  auto const& gi = impl_->gather_information_;

  gather_energy<dim><<<(this->mesh_->GetNumVertices() + GPU_GRAIN - 1) / GPU_GRAIN, GPU_GRAIN>>>(
      thrust::raw_pointer_cast(ev.data()), thrust::raw_pointer_cast(ee.data()),
      thrust::raw_pointer_cast(ge.data()), thrust::raw_pointer_cast(gi.data()),
      this->mesh_->GetNumVertices(), gi.size());
}

template <idx dim, template <idx> class ElasticModelTemplate>
void ElasticityCompute_GPU<dim, ElasticModelTemplate>::GatherStressToVertices() {
  TimeThisOne();
  auto& sv = impl_->stress_on_vertices_;
  auto const& se = impl_->stress_on_elements_;
  auto const& ge = impl_->gather_entries_;
  auto const& gi = impl_->gather_information_;
  auto const& ri = impl_->rinv_gpu_;
  gather_stress<dim><<<(this->mesh_->GetNumVertices() + GPU_GRAIN - 1) / GPU_GRAIN, GPU_GRAIN>>>(
      thrust::raw_pointer_cast(sv.data()), thrust::raw_pointer_cast(se.data()),
      thrust::raw_pointer_cast(impl_->rinv_gpu_.data()), thrust::raw_pointer_cast(ge.data()),
      thrust::raw_pointer_cast(gi.data()), this->mesh_->GetNumVertices(),
      this->mesh_->GetNumElements(), gi.size());
}

__device__ AX_FORCE_INLINE static math::matr<9, 12> ComputePFPx(const math::mat3r& DmInv) {
  const real m = DmInv(0, 0);
  const real n = DmInv(0, 1);
  const real o = DmInv(0, 2);
  const real p = DmInv(1, 0);
  const real q = DmInv(1, 1);
  const real r = DmInv(1, 2);
  const real s = DmInv(2, 0);
  const real t = DmInv(2, 1);
  const real u = DmInv(2, 2);
  const real t1 = -m - p - s;
  const real t2 = -n - q - t;
  const real t3 = -o - r - u;
  math::matr<9, 12> PFPx;
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
__device__ AX_FORCE_INLINE static math::matr<4, 6> ComputePFPx(const math::mat2r& DmInv) {
  const real m = DmInv(0, 0);
  const real n = DmInv(0, 1);
  const real p = DmInv(1, 0);
  const real q = DmInv(1, 1);
  const real t1 = -m - p;
  const real t2 = -n - q;
  math::matr<4, 6> PFPx;
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

template <idx dim>
__global__ void gather_hessian(COO* coo, math::veci<dim + 1> const* elements,
                               math::matr<dim, dim> const* rinv,
                               math::matr<dim * dim, dim * dim> const* hessian_on_elements,
                               idx n_elem) {
  idx eid = blockIdx.x * blockDim.x + threadIdx.x;
  if (eid > n_elem) return;

  idx coo_offset = eid * (dim + 1) * (dim + 1) * dim * dim;
  math::matr<dim * dim, dim * dim> const& H = hessian_on_elements[eid];
  math::matr<dim, dim> const& DmInv = rinv[eid];
  math::matr<dim * dim, dim*(dim + 1)> PFPx = ComputePFPx(DmInv);
  math::matr<(dim + 1) * dim, dim*(dim + 1)> pppx = PFPx.transpose() * H * PFPx;
  math::veci<dim + 1> ijk = elements[eid];

  // printf("e%ld H00=%lf", eid, H(0, 0));
  for (idx i = 0; i <= dim; ++i) {
    for (idx j = 0; j <= dim; ++j) {
      idx i_idx = ijk[i];
      idx j_idx = ijk[j];
      for (idx di = 0; di < dim; ++di) {
        for (idx dj = 0; dj < dim; ++dj) {
          idx coo_i = (i * (dim + 1) + j) * dim * dim + di * dim + dj + coo_offset;
          coo[coo_i].row = i_idx * dim + di;
          coo[coo_i].col = j_idx * dim + dj;
          coo[coo_i].val = pppx(i * dim + di, j * dim + dj);
        }
      }
    }
  }
}

template <idx dim, template <idx> class ElasticModelTemplate>
void ElasticityCompute_GPU<dim, ElasticModelTemplate>::GatherHessianToVertices() {
  TimeThisOne();
  auto const ne = this->mesh_->GetNumElements();
  gather_hessian<dim>
      <<<(ne + GPU_GRAIN - 1) / GPU_GRAIN, GPU_GRAIN>>>(thrust::raw_pointer_cast(impl_->hessian_on_vertices_.data()),
                                  thrust::raw_pointer_cast(impl_->elements_.data()),
                                  thrust::raw_pointer_cast(impl_->rinv_gpu_.data()),
                                  thrust::raw_pointer_cast(impl_->hessian_on_elements_.data()), ne);
}

template <idx dim, template <idx> class ElasticModelTemplate>
math::field1r const& ElasticityCompute_GPU<dim, ElasticModelTemplate>::GetEnergyOnVertices() {
  // Copy back:
  auto& gpu = impl_->energy_on_vertices_;
  auto& cpu = this->energy_on_vertices_;
  cudaMemcpy(cpu.data(), thrust::raw_pointer_cast(gpu.data()), cpu.size() * sizeof(real),
             cudaMemcpyDeviceToHost);
  return cpu;
}

template <idx dim, template <idx> class ElasticModelTemplate>
math::field1r const& ElasticityCompute_GPU<dim, ElasticModelTemplate>::GetEnergyOnElements() {
  auto& gpu = impl_->energy_on_elements_;
  auto& cpu = this->energy_on_elements_;
  cudaMemcpy(cpu.data(), thrust::raw_pointer_cast(gpu.data()), cpu.size() * sizeof(real),
             cudaMemcpyDeviceToHost);
  return cpu;
}

template <idx dim, template <idx> class ElasticModelTemplate>
math::fieldr<dim> const& ElasticityCompute_GPU<dim, ElasticModelTemplate>::GetStressOnVertices() {
  auto& gpu = impl_->stress_on_vertices_;
  auto& cpu = this->stress_on_vertices_;
  thrust::host_vector<math::vecr<dim>> cpu_vec = gpu;
  for (idx i = 0; i < (idx)cpu_vec.size(); ++i) {
    cpu.col(i) = cpu_vec[i];
  }
  return cpu;
}

template <idx dim, template <idx> class ElasticModelTemplate> List<math::matr<dim, dim>> const&
ElasticityCompute_GPU<dim, ElasticModelTemplate>::GetStressOnElements() {
  auto& gpu = impl_->stress_on_elements_;
  auto& cpu = this->stress_on_elements_;
  thrust::copy(gpu.begin(), gpu.end(), cpu.begin());
  return cpu;
}

template <idx dim, template <idx> class ElasticModelTemplate>
List<math::matr<dim * dim, dim * dim>> const&
ElasticityCompute_GPU<dim, ElasticModelTemplate>::GetHessianOnElements() {
  auto& gpu = impl_->hessian_on_elements_;
  auto& cpu = this->hessian_on_elements_;
  thrust::copy(gpu.begin(), gpu.end(), cpu.begin());
  return cpu;
}

template <idx dim, template <idx> class ElasticModelTemplate>
math::sp_matxxr const& ElasticityCompute_GPU<dim, ElasticModelTemplate>::GetHessianOnVertices() {
  auto& gpu = impl_->hessian_on_vertices_;
  auto& cpu = this->hessian_on_vertices_;
  thrust::host_vector<COO> coo = gpu;
  math::sp_coeff_list c;
  c.reserve(coo.size());
  for (auto I : coo) {
    c.push_back({I.row, I.col, I.val});
  }
  idx ndof = dim * this->mesh_->GetNumVertices();
  cpu = math::make_sparse_matrix(ndof, ndof, c);
  return cpu;
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
template class ElasticityCompute_GPU<3, elasticity::IsotropicARAP>;

}  // namespace ax::fem
