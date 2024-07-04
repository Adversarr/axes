#include <Eigen/SVD>

#include "ax/fem/elasticity.hpp"
#include "ax/fem/elasticity/arap.hpp"
#include "ax/fem/elasticity/linear.hpp"
#include "ax/fem/elasticity/neohookean_bw.hpp"
#include "ax/fem/elasticity/stable_neohookean.hpp"
#include "ax/fem/elasticity/stvk.hpp"
#include "ax/math/decomp/svd/import_eigen.hpp"
#include "ax/math/decomp/svd/remove_rotation.hpp"
#include "ax/optim/spsdm/eigenvalue.hpp"
#include "ax/utils/iota.hpp"


#include <tbb/partitioner.h>
#include <tbb/parallel_for.h>

// TBB States that, The relation between GRAINSIZE and INSTRUCTION COUNT should be:
//    G * IC >= 100,000
// We take Linear Elasticity as a reference:
// 1. EnergyImpl computation: F-norm of E and trace E, E is from a F+F.T - I, with several scalar
// operations.
//    About 50 instructions per element(also take account of the memory ops), we set G = 4000.
// 2. StressImpl computation: 1nd Piola-Kirchhoff stress, which is a 3x3 matrix, with 9 scalar
// operations.
//    Yet about 50 instructions per element, but for most nonlinears, log, det is required, the
//    instruction count is over 100, we set G = 1500.
// 3. HessianImpl, typically the most difficult, many instructions are memory associated. we cannot
// expect it to be
//    as fast as StressImpl computation. We set G = 500, this should be a reasonable value.
// This value shoule performs well for most time consuming elasticity energies, such as Isotropic
// ARAP. NOTE: Manually set the GRAIN SIZE if you have a different model.

#ifndef AX_FEM_COMPUTE_SVD_ALGORITHM
#  define AX_FEM_COMPUTE_SVD_ALGORITHM ax::math::decomp::JacobiSvd
#endif

#ifndef AX_FEM_COMPUTE_ENERGY_GRAIN
#  define AX_FEM_COMPUTE_ENERGY_GRAIN 6000
#endif

#ifndef AX_FEM_COMPUTE_STRESS_GRAIN
#  define AX_FEM_COMPUTE_STRESS_GRAIN 4000
#endif

#ifndef AX_FEM_COMPUTE_HESSIAN_GRAIN
#  define AX_FEM_COMPUTE_HESSIAN_GRAIN 2000
#endif

namespace ax::fem {

template <idx dim, template <idx> class ElasticModelTemplate>
struct ElasticityCompute_CPU<dim, ElasticModelTemplate>::TbbPartitioners {
  tbb::affinity_partitioner e_ap, h_ap, s_ap, svd_ap;
};

template <idx dim> static bool check_cache(elasticity::DeformationGradientCache<dim> const& cache) {
  bool has_error = false;
  for (const auto& R_inv : cache) {
    real detR = R_inv.determinant();
    if (math::isnan(detR) || math::abs(detR) < math::epsilon<real>) {
      AX_LOG(ERROR) << "Found Determinant of R.inv is nan, or det is nearly zero.";
      has_error = true;
    }
  }
  return !has_error;
}

template <idx dim>
static math::field1r dg_tev_p1(TriMesh<dim> const& mesh_, math::field1r const& e) {
  idx n_element = mesh_.GetNumElements();
  math::field1r result(1, mesh_.GetNumVertices());
  result.setZero();
  for (idx i = 0; i < n_element; ++i) {
    const auto& ijk = mesh_.GetElement(i);
    real energy = e[i] / real(dim + 1);
    for (idx I = 0; I <= dim; ++I) {
      result(ijk[I]) += energy;
    }
  }
  return result;
}

template <idx dim> typename TriMesh<dim>::vertex_list_t dg_tsv_p1(
    TriMesh<dim> const& mesh, List<elasticity::StressTensor<dim>> const& stress,
    elasticity::DeformationGradientCache<dim> const& cache) {
  typename TriMesh<dim>::vertex_list_t result;
  result.setZero(dim, mesh.GetNumVertices());
  // TODO: With Vertex->Element Map, the parallel is possible.
  for (idx i = 0; i < mesh.GetNumElements(); ++i) {
    // For P1 Element, the force on is easy to compute.
    const auto& ijk = mesh.GetElement(i);
    const auto& stress_i = stress[i];
    math::matr<dim, dim> R = cache[i];
    math::matr<dim, dim> f123 = stress_i * R.transpose();
    for (idx I = 1; I <= dim; ++I) {
      result.col(ijk[I]) += f123.col(I - 1);
      result.col(ijk.x()) -= f123.col(I - 1);
    }
  }
  return result;
}

/***********************************************************************************************
 * P1 Element Implementation.
 ***********************************************************************************************/
AX_FORCE_INLINE static math::matr<9, 12> ComputePFPx(const math::mat3r& DmInv) {
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
AX_FORCE_INLINE static math::matr<4, 6> ComputePFPx(const math::mat2r& DmInv) {
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
math::sp_coeff_list dg_thv_p1(TriMesh<dim> const& mesh,
                              List<elasticity::HessianTensor<dim>> const& hessian,
                              elasticity::DeformationGradientCache<dim> const& cache) {
  math::sp_coeff_list coo;
  coo.reserve(mesh.GetNumElements() * dim * dim * (dim + 1) * (dim + 1));
  std::vector<math::matr<dim*(dim + 1), dim*(dim + 1)>> per_element_hessian(mesh.GetNumElements());
  tbb::parallel_for(tbb::blocked_range<idx>(0, mesh.GetNumElements(), 500000 / dim * dim * dim),
                    [&](tbb::blocked_range<idx> const& r) {
                      for (idx i = r.begin(); i < r.end(); ++i) {
                        const auto& H_i = hessian[i];
                        math::matr<dim, dim> R = cache[i];
                        math::matr<dim * dim, dim*(dim + 1)> pfpx = ComputePFPx(R);
                        per_element_hessian[i] = pfpx.transpose() * H_i * pfpx;
                      }
                    });

  for (idx i = 0; i < mesh.GetNumElements(); ++i) {
    const auto& ijk = mesh.GetElement(i);
    const auto& H = per_element_hessian[i];
    for (auto [I, J, Di, Dj] : utils::multi_iota(dim + 1, dim + 1, dim, dim)) {
      idx i_idx = ijk[I];
      idx j_idx = ijk[J];
      idx H_idx_i = I * dim + Di;
      idx H_idx_j = J * dim + Dj;
      idx global_dof_i_idx = i_idx * dim + Di;
      idx global_dof_j_idx = j_idx * dim + Dj;
      coo.push_back({global_dof_i_idx, global_dof_j_idx, H(H_idx_i, H_idx_j)});
    }
  }
  return coo;
}

template <idx dim> ElasticityComputeBase<dim>::ElasticityComputeBase(SPtr<TriMesh<dim>> mesh)
    : mesh_(mesh), rinv_(static_cast<size_t>(mesh->GetNumElements())) {}

template <idx dim> void ElasticityComputeBase<dim>::SetMesh(const MeshPtr& mesh) {
  this->mesh_ = mesh;
  RecomputeRestPose();
}

template <idx dim> void ElasticityComputeBase<dim>::RecomputeRestPose() {
  // Prepare all the buffers.
  idx n_elem = mesh_->GetNumElements();
  idx n_vert = mesh_->GetNumVertices();

  energy_on_elements_.resize(1, n_elem);
  energy_on_vertices_.resize(1, n_vert);
  stress_on_elements_.resize(static_cast<size_t>(n_elem));
  stress_on_vertices_.resize(dim, n_vert);
  hessian_on_elements_.resize(static_cast<size_t>(n_elem));
}

template <idx dim> void ElasticityComputeBase<dim>::SetLame(math::vec2r const& u_lame) {
  lame_.resize(2, mesh_->GetNumElements());
  lame_.colwise() = u_lame;
}

template <idx dim> void ElasticityComputeBase<dim>::SetLame(math::field2r const& e_lame) {
  AX_CHECK(e_lame.cols() == mesh_->GetNumElements());
  lame_ = e_lame;
}

/*************************
 * SECT: CPU Implementation
 *************************/

template <idx dim, template <idx> class ElasticModelTemplate>
void ElasticityCompute_CPU<dim, ElasticModelTemplate>::UpdateEnergy() {
  idx const n_elem = this->mesh_->GetNumElements();
  auto const& dg_l = this->deformation_gradient_;
  const bool energy_requires_svd = ElasticModel().EnergyRequiresSvd();
  auto& element_energy = this->energy_on_elements_;
  tbb::parallel_for(
      tbb::blocked_range<idx>(0, n_elem, AX_FEM_COMPUTE_ENERGY_GRAIN),
      [&](const tbb::blocked_range<idx>& r) {
        ElasticModel model;
        math::decomp::SvdResult<dim, real> tomb;
        for (idx i = r.begin(); i < r.end(); ++i) {
          auto si = static_cast<size_t>(i);
          elasticity::DeformationGradient<dim> const& F = dg_l[si];
          model.SetLame(this->lame_.col(i));
          element_energy[i] = model.Energy(F, energy_requires_svd ? this->svd_results_[si] : tomb)
                              * this->rest_volume_(i);
        }
      },
      this->partitioner_impl_->e_ap);
}

template <idx dim, template <idx> class ElasticModelTemplate>
void ElasticityCompute_CPU<dim, ElasticModelTemplate>::UpdateStress() {
  idx const n_elem = this->mesh_->GetNumElements();
  auto const& dg_l = this->deformation_gradient_;
  auto const& lame = this->lame_;
  const bool stress_requires_svd = ElasticModel().StressRequiresSvd();
  auto& stress = this->stress_on_elements_;
  tbb::parallel_for(
      tbb::blocked_range<idx>(0, n_elem, AX_FEM_COMPUTE_STRESS_GRAIN),
      [&](const tbb::blocked_range<idx>& r) {
        ElasticModel model;
        math::decomp::SvdResult<dim, real> tomb;
        for (idx i = r.begin(); i < r.end(); ++i) {
          size_t si = static_cast<size_t>(i);
          elasticity::DeformationGradient<dim> const& F = dg_l[si];
          model.SetLame(lame.col(i));
          stress[si] = model.Stress(F, stress_requires_svd ? this->svd_results_[si] : tomb)
                       * this->rest_volume_(i);
        }
      },
      this->partitioner_impl_->s_ap);
}

template <idx dim, template <idx> class ElasticModelTemplate>
void ElasticityCompute_CPU<dim, ElasticModelTemplate>::UpdateHessian(bool projection) {
  idx const n_elem = this->mesh_->GetNumElements();
  auto const& dg_l = this->deformation_gradient_;
  auto const& lame = this->lame_;
  const bool hessian_requires_svd = ElasticModel().HessianRequiresSvd();
  auto& hessian = this->hessian_on_elements_;
  tbb::parallel_for(
      tbb::blocked_range<idx>(0, n_elem, AX_FEM_COMPUTE_HESSIAN_GRAIN),
      [&](const tbb::blocked_range<idx>& r) {
        ElasticModel model;
        math::decomp::SvdResult<dim, real> tomb;
        for (idx i = r.begin(); i < r.end(); ++i) {
          size_t si = static_cast<size_t>(i);
          model.SetLame(lame.col(i));
          elasticity::DeformationGradient<dim> const& F = dg_l[si];
          hessian[si] = model.Hessian(F, hessian_requires_svd ? (this->svd_results_[si]) : tomb)
                       * this->rest_volume_(i);
          if (projection) {
            hessian[si] = optim::project_spd_by_eigvals<dim * dim>(hessian[si], 1e-4);
          }
        }
      },
      this->partitioner_impl_->h_ap);
}

template <idx dim, template <idx> class ElasticModelTemplate>
void ElasticityCompute_CPU<dim, ElasticModelTemplate>::GatherEnergyToVertices() {
  auto const& mesh = this->mesh_;
  auto const& e = this->energy_on_elements_;
  auto& result = this->energy_on_vertices_;
  idx n_element = mesh->GetNumElements();
  result.setZero();
  for (idx i = 0; i < n_element; ++i) {
    const auto& ijk = mesh->GetElement(i);
    real energy = e[i] / real(dim + 1);
    for (idx I = 0; I <= dim; ++I) {
      result(ijk[I]) += energy;
    }
  }
}

template <idx dim, template <idx> class ElasticModelTemplate>
void ElasticityCompute_CPU<dim, ElasticModelTemplate>::GatherStressToVertices() {
  auto& result = this->stress_on_vertices_; result.setZero();
  auto const& mesh = *(this->mesh_);
  auto const& cache = this->rinv_;
  auto const& stress = this->stress_on_elements_;
  // TODO: With Vertex->Element Map, the parallel is possible.
  for (idx i = 0; i < mesh.GetNumElements(); ++i) {
    // For P1 Element, the force on is easy to compute.
    auto const si = static_cast<size_t>(i);
    const auto& ijk = mesh.GetElement(i);
    const auto& stress_i = stress[static_cast<size_t>(si)];
    math::matr<dim, dim> R = cache[static_cast<size_t>(si)];
    math::matr<dim, dim> f123 = stress_i * R.transpose();
    for (idx I = 1; I <= dim; ++I) {
      result.col(ijk[I]) += f123.col(I - 1);
      result.col(ijk.x()) -= f123.col(I - 1);
    }
  }
}

template <idx dim, template <idx> class ElasticModelTemplate>
void ElasticityCompute_CPU<dim, ElasticModelTemplate>::GatherHessianToVertices() {
  math::sp_coeff_list coo;
  auto& result = this->hessian_on_vertices_;
  auto const& mesh = *(this->mesh_);
  auto const& cache = this->rinv_;
  auto const& hessian = this->hessian_on_elements_;
  size_t total = static_cast<size_t>(mesh.GetNumElements() * dim * dim * (dim + 1) * (dim + 1));
  size_t nE = static_cast<size_t>(mesh.GetNumElements());
  coo.reserve(total);
  std::vector<math::matr<dim*(dim + 1), dim*(dim + 1)>> per_element_hessian(nE);
  tbb::parallel_for(tbb::blocked_range<idx>(0, mesh.GetNumElements(), 500000 / dim * dim * dim),
                    [&](tbb::blocked_range<idx> const& r) {
                      for (idx i = r.begin(); i < r.end(); ++i) {
                        size_t const si = static_cast<size_t>(i);
                        const auto& H_i = hessian[si];
                        math::matr<dim, dim> R = cache[si];
                        math::matr<dim * dim, dim*(dim + 1)> pfpx = ComputePFPx(R);
                        per_element_hessian[si] = pfpx.transpose() * H_i * pfpx;
                      }
                    });

  for (idx i = 0; i < mesh.GetNumElements(); ++i) {
    const auto& ijk = mesh.GetElement(i);
    const auto& H = per_element_hessian[static_cast<size_t>(i)];
    for (auto [I, J, Di, Dj] : utils::multi_iota(dim + 1, dim + 1, dim, dim)) {
      idx i_idx = ijk[I];
      idx j_idx = ijk[J];
      idx H_idx_i = I * dim + Di;
      idx H_idx_j = J * dim + Dj;
      idx global_dof_i_idx = i_idx * dim + Di;
      idx global_dof_j_idx = j_idx * dim + Dj;
      coo.push_back({global_dof_i_idx, global_dof_j_idx, H(H_idx_i, H_idx_j)});
    }
  }
  idx n_vert = mesh.GetNumVertices();
  result = math::make_sparse_matrix(dim * n_vert, dim * n_vert, coo);
}

template <idx dim, template <idx> class ElasticModelTemplate>
bool ElasticityCompute_CPU<dim, ElasticModelTemplate>::Update(math::fieldr<dim> const& pose,
                                                              ElasticityUpdateLevel upt) {
  idx const n_elem = this->mesh_->GetNumElements();
  auto& dg_l = this->deformation_gradient_;
  auto& svd_results_ = this->svd_results_;
  dg_l.resize(static_cast<size_t>(n_elem));
  svd_results_.resize(static_cast<size_t>(n_elem));
  bool const need_svd
      = (ElasticModel{}.EnergyRequiresSvd() && upt == ElasticityUpdateLevel::kEnergy)
        || (ElasticModel{}.StressRequiresSvd() && upt == ElasticityUpdateLevel::kStress)
        || (ElasticModel{}.HessianRequiresSvd() && upt == ElasticityUpdateLevel::kHessian);
  std::atomic<bool> failed = false;
  tbb::parallel_for(
      tbb::blocked_range<idx>(0, n_elem, AX_FEM_COMPUTE_ENERGY_GRAIN),
      [&](const tbb::blocked_range<idx>& r) {
        AX_FEM_COMPUTE_SVD_ALGORITHM<dim, real> svd;
        for (idx i = r.begin(); i < r.end(); ++i) {
          auto const si = static_cast<size_t>(i);
          math::matr<dim, dim> Ds;
          math::veci<dim + 1> element = this->mesh_->GetElement(i);
          math::vecr<dim> local_zero = pose.col(element.x());
          for (idx I = 1; I <= dim; ++I) {
            Ds.col(I - 1) = pose.col(element[I]) - local_zero;
          }
          dg_l[si] = Ds * this->rinv_[si];
          if (need_svd) {
            svd_results_[si] = svd.Solve(dg_l[si]);
            math::decomp::svd_remove_rotation(svd_results_[si]);
          }
        }
      },
      this->partitioner_impl_->svd_ap);
  return failed.load();
}

template <idx dim, template <idx> class ElasticModelTemplate>
void ElasticityCompute_CPU<dim, ElasticModelTemplate>::RecomputeRestPose() {
  auto const& rest_pose = this->mesh_->GetVertices();
  idx const n_elem = this->mesh_->GetNumElements();
  this->rinv_.reserve(static_cast<size_t>(n_elem));
  real coef = dim == 3 ? 1.0 / 6.0 : 1.0 / 2.0;
  using namespace math;
  for (idx i = 0; i < n_elem; ++i) {
    matr<dim, dim> rest_local;
    const auto& element = this->mesh_->GetElement(i);
    const auto& local_zero = rest_pose.col(element.x());
    for (idx I = 1; I <= dim; ++I) {
      rest_local.col(I - 1) = rest_pose.col(element[I]) - local_zero;
    }
    this->rinv_[static_cast<size_t>(i)] = rest_local.inverse();
  }
  // There exist an efficient approximation to compute the volume:
  this->rest_volume_.resize(1, n_elem);
  for (idx i = 0; i < n_elem; ++i) {
    real v = coef / math::det(this->rinv_[static_cast<size_t>(i)]);
    this->rest_volume_(0, i) = abs(v);
  }

  // prepare all the buffers.
  ElasticityComputeBase<dim>::RecomputeRestPose();
}

template <idx dim, template <idx> class ElasticModelTemplate>
ElasticityCompute_CPU<dim, ElasticModelTemplate>::ElasticityCompute_CPU(SPtr<TriMesh<dim>> mesh)
    : ElasticityComputeBase<dim>(mesh) {
  this->partitioner_impl_ = std::make_unique<TbbPartitioners>();
}

template <idx dim, template <idx> class ElasticModelTemplate>
ElasticityCompute_CPU<dim, ElasticModelTemplate>::~ElasticityCompute_CPU(){}

template class ElasticityComputeBase<2>;
template class ElasticityComputeBase<3>;

template class ElasticityCompute_CPU<2, elasticity::StableNeoHookean>;
template class ElasticityCompute_CPU<2, elasticity::NeoHookeanBW>;
template class ElasticityCompute_CPU<2, elasticity::IsotropicARAP>;
template class ElasticityCompute_CPU<2, elasticity::StVK>;
template class ElasticityCompute_CPU<2, elasticity::Linear>;

template class ElasticityCompute_CPU<3, elasticity::StableNeoHookean>;
template class ElasticityCompute_CPU<3, elasticity::NeoHookeanBW>;
template class ElasticityCompute_CPU<3, elasticity::IsotropicARAP>;
template class ElasticityCompute_CPU<3, elasticity::StVK>;
template class ElasticityCompute_CPU<3, elasticity::Linear>;
}  // namespace ax::fem
