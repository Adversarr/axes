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

// just MSVC + NVCC requires this:
#ifdef _MSC_VER
#  include <complex>
template <typename T> auto arg(T x) { return std::arg(x); }
#endif

#include "ax/fem/elasticity_gpu.cuh"
#include "ax/math/decomp/svd/remove_rotation.hpp"
#include "ax/math/decomp/svd/svd_cuda.cuh"
#include "ax/utils/time.hpp"

namespace ax {

namespace fem {

template <Index dim> using SvdR = math::decomp::SvdResult<dim, real>;

struct VertexOnElementInfo {
  Index vert_id;
  Index elem_id;
  Index local_id;
  AX_HOST_DEVICE VertexOnElementInfo(Index v, Index e, Index l) : vert_id(v), elem_id(e), local_id(l) {}

  AX_HOST_DEVICE VertexOnElementInfo(const VertexOnElementInfo& other) = default;
};

struct HessianGatherInfo {
  Index i, j;                // row, col in hessian
  Index element_id;          // which element to fetch
  Index local_i, local_j;    // local index in the element
  Index local_di, local_dj;  // local index (dimension) in the element
};

struct SparseCOO {
  Index row;
  Index col;
  real val;
};

template <Index dim, template <Index> class ElasticModelTemplate>
struct ElasticityCompute_GPU<dim, ElasticModelTemplate>::Impl {
  thrust::device_vector<math::veci<dim + 1>> elements_;
  thrust::device_vector<math::RealVector<dim>> pose_gpu_;  ///< current pose
  thrust::device_vector<math::RealMatrix<dim, dim>> deformation_gradient_;
  thrust::device_vector<math::RealMatrix<dim, dim>> rinv_gpu_;
  thrust::device_vector<real> rest_volume_gpu_;
  thrust::device_vector<SvdR<dim>> svd_results_;

  thrust::device_vector<Index> gather_entries_;
  thrust::device_vector<VertexOnElementInfo> gather_information_;
  thrust::device_vector<Index> hessian_gather_entries_;
  thrust::device_vector<HessianGatherInfo> hessian_gather_info_;

  thrust::device_vector<math::RealMatrix<dim, dim>> stress_on_elements_;
  thrust::device_vector<math::RealVector<dim>> stress_on_vertices_;
  thrust::device_vector<math::RealMatrix<dim * dim, dim * dim>> hessian_on_elements_;
  thrust::device_vector<SparseCOO> hessian_on_vertices_;
  thrust::device_vector<real> hessian_on_vertices2_;
  thrust::device_vector<real> energy_on_vertices_;
  thrust::device_vector<real> energy_on_elements_;
  thrust::device_vector<math::RealVector2> lame_;

  thrust::host_vector<real> energy_on_vertices_cpu_;
  thrust::host_vector<real> energy_on_elements_cpu_;
};

#define GPU_GRAIN 64

template <Index dim>
__global__ void compute_deformation_gradient(math::veci<dim + 1> const* elements,
                                             math::RealVector<dim> const* pose,
                                             math::RealMatrix<dim, dim>* deformation_gradient,
                                             math::RealMatrix<dim, dim>* rinv, SvdR<dim>* svdr,
                                             Index n_elem, bool need_svd) {
  Index eid = blockIdx.x * blockDim.x + threadIdx.x;
  if (eid >= n_elem) return;

  const math::veci<dim + 1> elem = elements[eid];
  const math::RealVector<dim> x0 = pose[elem[0]];
  math::RealMatrix<dim, dim> Dm;
  for (Index i = 0; i < dim; ++i) {
    Dm.col(i) = pose[elem[i + 1]] - x0;
  }

  math::RealMatrix<dim, dim> F = Dm * rinv[eid];
  deformation_gradient[eid] = F;
  if (need_svd) {
    math::Matrix<f32, dim, dim> U, V, Ff;
    Ff = F.template cast<float>();
    math::Vector<f32, dim> S;
    math::decomp::svd(Ff, U, V, S);
    svdr[eid].U_ = U.template cast<real>();
    svdr[eid].V_ = V.template cast<real>();
    svdr[eid].sigma_ = S.template cast<real>();
    ax::math::decomp::svd_remove_rotation<ax::real>(svdr[eid]);
  }
}

template <Index dim>
__global__ void compute_rest_pose(math::veci<dim + 1> const* elements, math::RealVector<dim> const* pose,
                                  math::RealMatrix<dim, dim>* rinv, real* rest_volume, Index n_elem) {
  Index eid = blockIdx.x * blockDim.x + threadIdx.x;
  if (eid >= n_elem) return;

  const math::veci<dim + 1> elem = elements[eid];
  const math::RealVector<dim> x0 = pose[elem[0]];
  math::RealMatrix<dim, dim> Dm;
  for (Index i = 0; i < dim; ++i) {
    Dm.col(i) = pose[elem[i + 1]] - x0;
  }

  rinv[eid] = Dm.inverse();
  const real coef = dim == 3 ? 1.0 / 6.0 : 1.0 / 2.0;
  rest_volume[eid] = coef / fabs(math::det(rinv[eid]));
}

template <Index dim, template <Index> class ElasticModelTemplate>
ElasticityCompute_GPU<dim, ElasticModelTemplate>::ElasticityCompute_GPU(MeshPtr const& mesh)
    : ElasticityComputeBase<dim>(mesh) {
  cudaError_t error = cudaSetDevice(0);
  if (error != cudaSuccess) {
    std::cerr << "Error: " << cudaGetErrorString(error) << std::endl;
  }
  impl_ = std::make_unique<Impl>();
}

template <Index dim, template <Index> class ElasticModelTemplate>
ElasticityCompute_GPU<dim, ElasticModelTemplate>::~ElasticityCompute_GPU() {
  this->impl_.reset();
}

template <Index dim, template <Index> class ElasticModelTemplate>
void ElasticityCompute_GPU<dim, ElasticModelTemplate>::Update(math::RealField<dim> const& pose,
                                                              ElasticityUpdateLevel upt) {
  auto error = cudaMemcpy(thrust::raw_pointer_cast(impl_->pose_gpu_.data()), pose.data(),
                          pose.size() * sizeof(real), cudaMemcpyHostToDevice);
  // AX_CHECK(error == cudaSuccess) << "Failed to copy pose to GPU." << cudaGetErrorString(error);
  AX_CHECK(error == cudaSuccess, "Failed to copy pose to GPU: {}", cudaGetErrorString(error));
  Index n_elem = this->mesh_->GetNumElements();
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

  // thrust::host_vector<math::RealMatrix<dim, dim>> deformation_gradient_cpu(n_elem);
  // thrust::copy(impl_->deformation_gradient_.begin(), impl_->deformation_gradient_.end(),
  //              deformation_gradient_cpu.begin());
  // std::cout << "Deformation gradient: " << deformation_gradient_cpu[0] << std::endl;
}

template <Index dim, template <Index> class ElasticModelTemplate>
void ElasticityCompute_GPU<dim, ElasticModelTemplate>::RecomputeRestPose() {
  auto const& mesh = *(this->mesh_);
  Index n_elem = this->mesh_->GetNumElements();
  Index n_vert = this->mesh_->GetNumVertices();

  // Elements
  impl_->elements_.resize(n_elem);
  thrust::host_vector<math::veci<dim + 1>> elements_host(n_elem);
  for (Index eid = 0; eid < n_elem; ++eid) {
    auto elem = mesh.GetElement(eid);
    for (Index i = 0; i < dim + 1; ++i) {
      elements_host[eid][i] = elem[i];
    }
  }
  thrust::copy(elements_host.begin(), elements_host.end(), impl_->elements_.begin());

  impl_->rest_volume_gpu_.resize(n_elem);
  impl_->rinv_gpu_.resize(n_elem);
  impl_->deformation_gradient_.resize(n_elem);
  impl_->svd_results_.resize(n_elem);

  auto const& pose = mesh.GetVertices();
  thrust::host_vector<math::RealVector<dim>> pose_cpu(pose.cols());
  for (Index i = 0; i < pose.cols(); ++i) {
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

  // NOTE: Gathering information for energy and stress.
  thrust::host_vector<Index> gather_entries_cpu;
  thrust::host_vector<VertexOnElementInfo> info;
  for (Index e = 0; e < n_elem; ++e) {
    auto const& elem = mesh.GetElement(e);
    for (Index d = 0; d <= dim; ++d) {
      info.push_back(VertexOnElementInfo{elem[d], e, d});
    }
  }

  std::sort(info.begin(), info.end(),
            [](VertexOnElementInfo const& a, VertexOnElementInfo const& b) {
              return a.vert_id < b.vert_id;
            });

  gather_entries_cpu.resize(n_vert, -1);
  for (Index i = 0; (size_t)i < info.size(); ++i) {
    auto v = info[i].vert_id;
    if (gather_entries_cpu[v] == -1) {
      gather_entries_cpu[v] = i;
    }
  }

  impl_->gather_entries_ = gather_entries_cpu;
  impl_->gather_information_ = info;

  // TODO: Gathering information for Hessian
  thrust::host_vector<HessianGatherInfo> hgi;
  for (Index e = 0; e < n_elem; ++e) /* each element ... */ {
    auto const& elem = mesh.GetElement(e);
    for (Index i = 0; i <= dim; ++i) /* loop over vertices ... */ {
      for (Index j = 0; j <= dim; ++j) {
        Index vi = elem[i];
        Index vj = elem[j];
        for (Index di = 0; di < dim; ++di) /* loop over dimensions ... */ {
          for (Index dj = 0; dj < dim; ++dj) {
            HessianGatherInfo info;
            info.i = vi * dim + di;
            info.j = vj * dim + dj;
            info.element_id = e;
            info.local_i = i;
            info.local_j = j;
            info.local_di = di;
            info.local_dj = dj;
            hgi.push_back(info);
          }
        }
      }
    }
  }
  std::sort(hgi.begin(), hgi.end(),
            [n_vert](HessianGatherInfo const& a, HessianGatherInfo const& b) {
              Index ai = a.i * n_vert * dim + a.j;
              Index bi = b.i * n_vert * dim + b.j;
              return ai < bi;
            });

  thrust::host_vector<Index> hg_entries;
  hg_entries.push_back(0);
  for (size_t i = 1; i < hgi.size(); ++i) {
    if (hgi[i].i != hgi[i - 1].i || hgi[i].j != hgi[i - 1].j) {
      hg_entries.push_back(i);
    }
  }
  // for (Index i = 0; i < hg_entries.size(); ++i){
  //   Index ent = hg_entries[i];
  //   Index row = hgi[ent].i, col = hgi[ent].j;
  //   printf("Entry %ld: %ld => %ld %ld\n", i, ent, row, col);
  // }

  hg_entries.push_back(static_cast<Index>(hgi.size()));
  impl_->hessian_gather_info_ = hgi;
  impl_->hessian_gather_entries_ = hg_entries;
  Index nnz = hg_entries.size() - 1;
  impl_->hessian_on_vertices2_.resize(nnz);
  thrust::fill(impl_->hessian_on_vertices2_.begin(), impl_->hessian_on_vertices2_.end(), 0);

  // moreover, we need to setup the template sparse matrix!
  std::vector<math::sp_coeff> coeffs;
  for (auto const& info : hgi) {
    coeffs.push_back({info.i, info.j, 1});
  }
  this->hessian_on_vertices_ = math::make_sparse_matrix(n_vert * dim, n_vert * dim, coeffs);
  this->hessian_on_vertices_.makeCompressed();

  ElasticityComputeBase<dim>::RecomputeRestPose();
}

template <Index dim, template <Index> class ElasticModelTemplate>
void ElasticityCompute_GPU<dim, ElasticModelTemplate>::SetLame(math::RealField2 const& lame) {
  thrust::host_vector<math::RealVector2> lame_host(lame.cols());
  for (Index i = 0; i < lame.cols(); ++i) {
    lame_host[i] = lame.col(i);
  }
  impl_->lame_ = lame_host;
}

template <Index dim, template <Index> class ElasticModelTemplate>
void ElasticityCompute_GPU<dim, ElasticModelTemplate>::SetLame(math::RealVector2 const& lame) {
  thrust::host_vector<math::RealVector2> lame_host(this->mesh_->GetNumElements(), lame);
  impl_->lame_ = lame_host;
}

/*************************
 * SECT: Energy
 *************************/

template <Index dim, template <Index> class ElasticModelTemplate>
__global__ void compute_energy_impl(math::RealMatrix<dim, dim> const* deformation_gradient,
                                    real const* rest_volume, SvdR<dim> const* svd_results,
                                    real* energy, math::RealVector2 const* lame, Index n_elem) {
  Index eid = blockIdx.x * blockDim.x + threadIdx.x;
  if (eid >= n_elem) return;
  // printf("E begin: Lame of %ld is %lf %lf\n", eid, lame[eid][0], lame[eid][1]);
  ElasticModelTemplate<dim> model(lame[eid][0], lame[eid][1]);
  energy[eid] = model.Energy(deformation_gradient[eid], svd_results[eid]) * rest_volume[eid];
  // printf("E end: Lame of %ld is %lf %lf\n", eid, lame[eid][0], lame[eid][1]);
}

/*************************
 * SECT: Stress
 *************************/
template <Index dim, template <Index> class ElasticModelTemplate>
__global__ void compute_stress_impl(math::RealMatrix<dim, dim> const* deformation_gradient,
                                    real const* rest_volume, SvdR<dim> const* svd_results,
                                    math::RealVector2 const* lame, elasticity::StressTensor<dim>* stress,
                                    Index n_elem) {
  size_t eid = blockIdx.x * blockDim.x + threadIdx.x;
  if (eid >= n_elem) return;
  ElasticModelTemplate<dim> model(lame[eid][0], lame[eid][1]);
  stress[eid] = model.Stress(deformation_gradient[eid], svd_results[eid]) * rest_volume[eid];
  // printf("Lame of %ld is %lf %lf\n", eid, lame[eid][0], lame[eid][1]);
}

/*************************
 * SECT: Hessian
 *************************/
template <Index dim, template <Index> class ElasticModelTemplate>
__global__ void compute_hessian_impl(math::RealMatrix<dim, dim> const* deformation_gradient,
                                     real const* rest_volume, SvdR<dim> const* svd_results,
                                     elasticity::HessianTensor<dim>* hessian,
                                     math::RealVector2 const* lame, Index n_elem, bool projection) {
  Index eid = blockIdx.x * blockDim.x + threadIdx.x;
  if (eid >= n_elem) return;
  // printf("H begin: Lame of %ld is %lf %lf\n", eid, lame[eid][0], lame[eid][1]);

  ElasticModelTemplate<dim> model(lame[eid][0], lame[eid][1]);
  hessian[eid] = model.Hessian(deformation_gradient[eid], svd_results[eid]) * rest_volume[eid];

  if (projection) {
    math::RealMatrix<dim * dim, dim * dim>& H = hessian[eid];
    Eigen::SelfAdjointEigenSolver<math::RealMatrix<dim * dim, dim * dim>> es(H);
    math::RealVector<dim * dim> D = es.eigenvalues().cwiseMax(1e-4);
    math::RealMatrix<dim * dim, dim * dim> V = es.eigenvectors();
    H = V * D.asDiagonal() * V.transpose();
  }

  // printf("H end: Lame of %ld is %lf %lf\n", eid, lame[eid][0], lame[eid][1]);
}

template <Index dim, template <Index> class ElasticModelTemplate>
void ElasticityCompute_GPU<dim, ElasticModelTemplate>::UpdateEnergy() {
  auto& energy_on_elements = impl_->energy_on_elements_;
  auto const& lame = impl_->lame_;
  auto const& F = impl_->deformation_gradient_;
  auto const& rest_volume = impl_->rest_volume_gpu_;
  auto const& svd = impl_->svd_results_;

  compute_energy_impl<dim, ElasticModelTemplate>
      <<<(impl_->deformation_gradient_.size() + GPU_GRAIN - 1) / GPU_GRAIN, GPU_GRAIN>>>(
          thrust::raw_pointer_cast(impl_->deformation_gradient_.data()),
          thrust::raw_pointer_cast(impl_->rest_volume_gpu_.data()),
          thrust::raw_pointer_cast(impl_->svd_results_.data()),
          thrust::raw_pointer_cast(impl_->energy_on_elements_.data()),
          thrust::raw_pointer_cast(impl_->lame_.data()), impl_->deformation_gradient_.size());
}

template <Index dim, template <Index> class ElasticModelTemplate>
void ElasticityCompute_GPU<dim, ElasticModelTemplate>::UpdateStress() {
  auto& stress_on_elements = impl_->stress_on_elements_;
  auto const& lame = impl_->lame_;
  auto const& F = impl_->deformation_gradient_;
  auto const& rest_volume = impl_->rest_volume_gpu_;
  auto const& svd = impl_->svd_results_;

  compute_stress_impl<dim, ElasticModelTemplate>
      <<<(impl_->deformation_gradient_.size() + GPU_GRAIN - 1) / GPU_GRAIN, GPU_GRAIN>>>(
          thrust::raw_pointer_cast(impl_->deformation_gradient_.data()),
          thrust::raw_pointer_cast(impl_->rest_volume_gpu_.data()),
          thrust::raw_pointer_cast(impl_->svd_results_.data()),
          thrust::raw_pointer_cast(impl_->lame_.data()),
          thrust::raw_pointer_cast(impl_->stress_on_elements_.data()),
          impl_->deformation_gradient_.size());
}

template <Index dim, template <Index> class ElasticModelTemplate>
void ElasticityCompute_GPU<dim, ElasticModelTemplate>::UpdateHessian(bool projection) {
  auto& hessian_on_elements = impl_->hessian_on_elements_;
  auto const& lame = impl_->lame_;
  auto const& F = impl_->deformation_gradient_;
  auto const& rest_volume = impl_->rest_volume_gpu_;
  auto const& svd = impl_->svd_results_;

  compute_hessian_impl<dim, ElasticModelTemplate>
      <<<(impl_->deformation_gradient_.size() + GPU_GRAIN - 1) / GPU_GRAIN, GPU_GRAIN>>>(
          thrust::raw_pointer_cast(impl_->deformation_gradient_.data()),
          thrust::raw_pointer_cast(impl_->rest_volume_gpu_.data()),
          thrust::raw_pointer_cast(impl_->svd_results_.data()),
          thrust::raw_pointer_cast(impl_->hessian_on_elements_.data()),
          thrust::raw_pointer_cast(impl_->lame_.data()), impl_->deformation_gradient_.size(),
          projection);
}

template <Index dim> __global__ void gather_energy(real* energy_on_vertices,
                                                 real const* energy_on_elements, Index const* entry,
                                                 VertexOnElementInfo const* gather_info, Index n_vert,
                                                 Index n_info) {
  Index vid = blockIdx.x * blockDim.x + threadIdx.x;
  if (vid >= n_vert) return;
  Index e = entry[vid];
  real total_energy = 0;
  if (e == -1) {
    energy_on_vertices[vid] = 0;
    return;
  }
  for (Index i = e; i < n_info; ++i) {
    if (gather_info[i].vert_id == vid) {
      total_energy += energy_on_elements[gather_info[i].elem_id];
    } else {
      break;
    }
  }
  energy_on_vertices[vid] = total_energy / real(dim + 1);
}

template <Index dim> __global__ void gather_stress(math::RealVector<dim>* stress_on_vertices,
                                                 math::RealMatrix<dim, dim> const* stress_on_elements,
                                                 math::RealMatrix<dim, dim> const* rinv, Index const* entry,
                                                 VertexOnElementInfo const* gather_info, Index n_vert,
                                                 Index n_elem, Index n_info) {
  const Index vid = blockIdx.x * blockDim.x + threadIdx.x;
  if (vid >= n_vert) return;
  Index const e = entry[vid];
  math::RealVector<dim> total;
  total.setZero();
  if (e == -1) {
    stress_on_vertices[vid] = total;
    return;
  }
  for (Index i = e; i < n_info; ++i) {
    if (gather_info[i].vert_id == vid) {
      const Index eid = gather_info[i].elem_id;
      math::RealMatrix<dim, dim> const& Dm_inv = rinv[eid];
      math::RealMatrix<dim, dim> const& Pk1 = stress_on_elements[eid];
      math::RealMatrix<dim, dim> Pk = Pk1 * Dm_inv.transpose();
      if (gather_info[i].local_id == 0) {
        for (Index D = 0; D < dim; ++D) {
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

template <Index dim, template <Index> class ElasticModelTemplate>
void ElasticityCompute_GPU<dim, ElasticModelTemplate>::GatherEnergyToVertices() {
  auto& ev = impl_->energy_on_vertices_;
  auto const& ee = impl_->energy_on_elements_;
  auto const& ge = impl_->gather_entries_;
  auto const& gi = impl_->gather_information_;

  gather_energy<dim><<<(this->mesh_->GetNumVertices() + GPU_GRAIN - 1) / GPU_GRAIN, GPU_GRAIN>>>(
      thrust::raw_pointer_cast(ev.data()), thrust::raw_pointer_cast(ee.data()),
      thrust::raw_pointer_cast(ge.data()), thrust::raw_pointer_cast(gi.data()),
      this->mesh_->GetNumVertices(), gi.size());
}

template <Index dim, template <Index> class ElasticModelTemplate>
void ElasticityCompute_GPU<dim, ElasticModelTemplate>::GatherStressToVertices() {
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

__device__ AX_FORCE_INLINE static math::RealMatrix<9, 12> ComputePFPx(const math::RealMatrix3& DmInv) {
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
__device__ AX_FORCE_INLINE static math::RealMatrix<4, 6> ComputePFPx(const math::RealMatrix2& DmInv) {
  const real m = DmInv(0, 0);
  const real n = DmInv(0, 1);
  const real p = DmInv(1, 0);
  const real q = DmInv(1, 1);
  const real t1 = -m - p;
  const real t2 = -n - q;
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

template <Index dim> __device__ Index coo_locate_hessian(Index elem_id, Index i, Index j, Index di, Index dj) {
  Index coo_offset = elem_id * (dim + 1) * dim * (dim + 1) * dim;
  Index local_offset = (i * (dim + 1) + j) * dim * dim + di * dim + dj;
  return coo_offset + local_offset;
}

template <Index dim>
__global__ void gather_hessian(SparseCOO* coo, math::veci<dim + 1> const* elements,
                               math::RealMatrix<dim, dim> const* rinv,
                               math::RealMatrix<dim * dim, dim * dim> const* hessian_on_elements,
                               Index n_elem) {
  Index eid = blockIdx.x * blockDim.x + threadIdx.x;
  if (eid >= n_elem) return;

  math::RealMatrix<dim * dim, dim * dim> const& H = hessian_on_elements[eid];
  math::RealMatrix<dim, dim> const& DmInv = rinv[eid];
  math::RealMatrix<dim * dim, dim*(dim + 1)> PFPx = ComputePFPx(DmInv);
  math::RealMatrix<(dim + 1) * dim, dim*(dim + 1)> pppx = PFPx.transpose() * H * PFPx;
  math::veci<dim + 1> ijk = elements[eid];

  for (Index i = 0; i <= dim; ++i) {
    for (Index j = 0; j <= dim; ++j) {
      Index i_Index = ijk[i];
      Index j_Index = ijk[j];
      for (Index di = 0; di < dim; ++di) {
        for (Index dj = 0; dj < dim; ++dj) {
          Index coo_i = coo_locate_hessian<dim>(eid, i, j, di, dj);
          coo[coo_i].row = i_Index * dim + di;
          coo[coo_i].col = j_Index * dim + dj;
          coo[coo_i].val = pppx(i * dim + di, j * dim + dj);
        }
      }
    }
  }
}

template <Index dim, template <Index> class ElasticModelTemplate>
void ElasticityCompute_GPU<dim, ElasticModelTemplate>::GatherHessianToVertices() {
  auto const ne = this->mesh_->GetNumElements();
  gather_hessian<dim><<<(ne + GPU_GRAIN - 1) / GPU_GRAIN, GPU_GRAIN>>>(
      thrust::raw_pointer_cast(impl_->hessian_on_vertices_.data()),
      thrust::raw_pointer_cast(impl_->elements_.data()),
      thrust::raw_pointer_cast(impl_->rinv_gpu_.data()),
      thrust::raw_pointer_cast(impl_->hessian_on_elements_.data()), ne);
}

template <Index dim, template <Index> class ElasticModelTemplate>
math::RealField1 const& ElasticityCompute_GPU<dim, ElasticModelTemplate>::GetEnergyOnVertices() {
  auto& gpu = impl_->energy_on_vertices_;
  auto& cpu = impl_->energy_on_vertices_cpu_;
  auto& ret = this->energy_on_vertices_;

  cpu = gpu;
  memcpy(ret.data(), thrust::raw_pointer_cast(cpu.data()), ret.size() * sizeof(real));
  return ret;
}

template <Index dim, template <Index> class ElasticModelTemplate>
math::RealField1 const& ElasticityCompute_GPU<dim, ElasticModelTemplate>::GetEnergyOnElements() {
  auto& gpu = impl_->energy_on_elements_;
  auto& cpu = impl_->energy_on_elements_cpu_;
  auto& ret = this->energy_on_elements_;
  cpu = gpu;
  memcpy(ret.data(), thrust::raw_pointer_cast(cpu.data()), ret.size() * sizeof(real));
  return ret;
}

template <Index dim, template <Index> class ElasticModelTemplate>
math::RealField<dim> const& ElasticityCompute_GPU<dim, ElasticModelTemplate>::GetStressOnVertices() {
  auto& gpu = impl_->stress_on_vertices_;
  auto& cpu = this->stress_on_vertices_;
  thrust::host_vector<math::RealVector<dim>> cpu_vec = gpu;
  for (Index i = 0; i < (Index)cpu_vec.size(); ++i) {
    cpu.col(i) = cpu_vec[i];
  }
  return cpu;
}

template <Index dim, template <Index> class ElasticModelTemplate>
elasticity::vector_for_eigen_type<math::RealMatrix<dim, dim>> const&
ElasticityCompute_GPU<dim, ElasticModelTemplate>::GetStressOnElements() {
  auto& gpu = impl_->stress_on_elements_;
  auto& cpu = this->stress_on_elements_;
  thrust::copy(gpu.begin(), gpu.end(), cpu.begin());
  return cpu;
}

template <Index dim, template <Index> class ElasticModelTemplate>
elasticity::vector_for_eigen_type<math::RealMatrix<dim * dim, dim * dim>> const&
ElasticityCompute_GPU<dim, ElasticModelTemplate>::GetHessianOnElements() {
  auto& gpu = impl_->hessian_on_elements_;
  auto& cpu = this->hessian_on_elements_;
  thrust::copy(gpu.begin(), gpu.end(), cpu.begin());
  return cpu;
}

template <Index dim>
__global__ void gather_hessian2(real* dst, HessianGatherInfo* hessian_gather_info,
                                Index* hessian_gather_entries, SparseCOO* hessian_on_vertices,
                                Index max_entries) {
  Index i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= max_entries) return;
  dst[i] = 0;
  for (Index entry = hessian_gather_entries[i]; entry < hessian_gather_entries[i + 1]; ++entry) {
    auto& hgi = hessian_gather_info[entry];
    Index row = hgi.i, col = hgi.j;
    Index element_id = hgi.element_id;
    Index local_i = hgi.local_i;
    Index local_j = hgi.local_j;
    Index local_di = hgi.local_di;
    Index local_dj = hgi.local_dj;
    Index coo = coo_locate_hessian<dim>(element_id, local_i, local_j, local_di, local_dj);

    SparseCOO& coo_entry = hessian_on_vertices[coo];
    Index row_coo = coo_entry.row, col_coo = coo_entry.col;

    // NOTE: check
    // if (row == row_coo && col == col_coo) {
    dst[i] += hessian_on_vertices[coo].val;
    // } else {
    //   printf("Error: i=%ld, row=%ld, col=%ld, row_coo=%ld, col_coo=%ld\n", i, row, col, row_coo,
    //   col_coo);
    // }
  }
}

template <Index dim, template <Index> class ElasticModelTemplate>
math::spmatr const& ElasticityCompute_GPU<dim, ElasticModelTemplate>::GetHessianOnVertices() {
  auto& gpu = impl_->hessian_on_vertices_;
  auto& cpu = this->hessian_on_vertices_;
  Index ndof = dim * this->mesh_->GetNumVertices();

  Index nnz = impl_->hessian_gather_entries_.size() - 1;
  gather_hessian2<dim><<<(nnz + GPU_GRAIN - 1) / GPU_GRAIN, GPU_GRAIN>>>(
      thrust::raw_pointer_cast(impl_->hessian_on_vertices2_.data()),
      thrust::raw_pointer_cast(impl_->hessian_gather_info_.data()),
      thrust::raw_pointer_cast(impl_->hessian_gather_entries_.data()),
      thrust::raw_pointer_cast(impl_->hessian_on_vertices_.data()), nnz);

  real* value = cpu.valuePtr();
  thrust::copy(impl_->hessian_on_vertices2_.begin(), impl_->hessian_on_vertices2_.end(), value);
  // // NOTE: check the correctness
  // thrust::host_vector<SparseCOO> coo = gpu;
  // math::sp_coeff_list c;
  // c.reserve(coo.size());
  // for (auto I : coo) {
  //   c.push_back({I.row, I.col, I.val});
  // }
  // auto cpu2 = math::make_sparse_matrix(ndof, ndof, c);
  // for (Index i = 0; i < ndof; ++i) {
  //   for (math::spmatr::InnerIterator it(cpu2, i); it; ++it) {
  //     Index j = it.col();
  //     real v = it.value();
  //     if (fabs(v - cpu.coeff(i, j)) > 1e-6) {
  //       printf("Error: %ld %ld %lf %lf\n", i, j, v, cpu.coeff(i, j));
  //     }
  //   }
  // }

  return cpu;
}

// template class ElasticityCompute_GPU<2, elasticity::StableNeoHookean>;
// template class ElasticityCompute_GPU<2, elasticity::NeoHookeanBW>;
// template class ElasticityCompute_GPU<2, elasticity::StVK>;
// template class ElasticityCompute_GPU<2, elasticity::Linear>;
// template class ElasticityCompute_GPU<2, elasticity::IsotropicARAP>;
//
// template class ElasticityCompute_GPU<3, elasticity::StableNeoHookean>;
// template class ElasticityCompute_GPU<3, elasticity::NeoHookeanBW>;
// template class ElasticityCompute_GPU<3, elasticity::StVK>;
// template class ElasticityCompute_GPU<3, elasticity::Linear>;
// template class ElasticityCompute_GPU<3, elasticity::IsotropicARAP>;

}  // namespace fem
}  // namespace ax