#include "ax/fem/elasticity_cpu.hpp"

#include <tbb/parallel_for.h>
#include <tbb/partitioner.h>

#include "ax/fem/elasticity/arap.hpp"
#include "ax/fem/elasticity/linear.hpp"
#include "ax/fem/elasticity/neohookean_bw.hpp"
#include "ax/fem/elasticity/stable_neohookean.hpp"
#include "ax/fem/elasticity/stvk.hpp"
#include "ax/math/decomp/svd/import_eigen.hpp"
#include "ax/math/decomp/svd/remove_rotation.hpp"
#include "ax/math/shape.hpp"
#include "ax/math/views.hpp"
#include "ax/optim/spsdm/eigenvalue.hpp"
#include "ax/utils/ndrange.hpp"
#include "taskflow/algorithm/for_each.hpp"

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
#  define AX_FEM_COMPUTE_ENERGY_GRAIN 600
#endif

#ifndef AX_FEM_COMPUTE_STRESS_GRAIN
#  define AX_FEM_COMPUTE_STRESS_GRAIN 400
#endif

#ifndef AX_FEM_COMPUTE_HESSIAN_GRAIN
#  define AX_FEM_COMPUTE_HESSIAN_GRAIN 200
#endif

namespace ax::fem {

struct HessianEntryInfo {
  // The local Hessian
  Index element_id_;
  Index local_i_;  // range 0..dof (dof = dim*(dim+1))
  Index local_j_;  // range 0..dof
  // The global Hessian
  Index global_i_;  // range 0..n_vert * dim
  Index global_j_;  // range 0..n_vert * dim (i.e. global dof)
};

// Since we are using ColMajor sparse matrix:
static bool prec(Index dofs, HessianEntryInfo const& a, HessianEntryInfo const& b) {
  return a.global_j_ * dofs + a.global_i_ < b.global_j_ * dofs + b.global_i_;
}

template <int dim, template <int> class ElasticModelTemplate>
struct ElasticityCompute_CPU<dim, ElasticModelTemplate>::Impl {
  tbb::affinity_partitioner e_ap, h_ap, s_ap, svd_ap;

  std::vector<HessianEntryInfo> hessian_entries_;
  std::vector<size_t> outer_start_;

  tf::Executor executor_;
};

template <int dim>
static bool check_cache(elasticity::DeformGradCache<dim> const& cache) {
  bool has_error = false;
  for (const auto& R_inv : cache) {
    Real detR = R_inv.determinant();
    if (math::isnan(detR) || math::abs(detR) < math::epsilon<Real>) {
      AX_ERROR("Found Determinant of R.inv is nan, or det is nearly zero.");
      has_error = true;
    }
  }
  return !has_error;
}

template <int dim>
static math::RealField1 dg_tev_p1(TriMesh<dim> const& mesh_, math::RealField1 const& e) {
  Index n_element = mesh_.GetNumElements();
  math::RealField1 result(1, mesh_.GetNumVertices());
  result.setZero();
  for (Index i = 0; i < n_element; ++i) {
    const auto& ijk = mesh_.GetElement(i);
    Real energy = e[i] / Real(dim + 1);
    for (Index I = 0; I <= dim; ++I) {
      result(ijk[I]) += energy;
    }
  }
  return result;
}

template <int dim>
typename TriMesh<dim>::vertex_list_t dg_tsv_p1(
    TriMesh<dim> const& mesh, std::vector<elasticity::StressTensor<dim>> const& stress,
    elasticity::DeformGradCache<dim> const& cache) {
  typename TriMesh<dim>::vertex_list_t result;
  result.setZero(dim, mesh.GetNumVertices());
  // TODO: With Vertex->Element Map, the parallel is possible.
  for (Index i = 0; i < mesh.GetNumElements(); ++i) {
    // For P1 Element, the force on is easy to compute.
    const auto& ijk = mesh.GetElement(i);
    const auto& stress_i = stress[i];
    math::RealMatrix<dim, dim> R = cache[i];
    math::RealMatrix<dim, dim> f123 = stress_i * R.transpose();
    for (Index I = 1; I <= dim; ++I) {
      result.col(ijk[I]) += f123.col(I - 1);
      result.col(ijk.x()) -= f123.col(I - 1);
    }
  }
  return result;
}

/***********************************************************************************************
 * P1 Element Implementation.
 ***********************************************************************************************/
AX_FORCE_INLINE static math::RealMatrix<9, 12> ComputePFPx(const math::RealMatrix3& DmInv) {
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
AX_FORCE_INLINE static math::RealMatrix<4, 6> ComputePFPx(const math::RealMatrix2& DmInv) {
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

template <int dim>
math::SparseCOO dg_thv_p1(TriMesh<dim> const& mesh,
                          std::vector<elasticity::HessianTensor<dim>> const& hessian,
                          elasticity::DeformGradCache<dim> const& cache) {
  math::SparseCOO coo;
  coo.reserve(mesh.GetNumElements() * dim * dim * (dim + 1) * (dim + 1));
  std::vector<math::RealMatrix<dim*(dim + 1), dim*(dim + 1)>> per_element_hessian(
      mesh.GetNumElements());
  tbb::parallel_for(tbb::blocked_range<Index>(0, mesh.GetNumElements(), 500000 / dim * dim * dim),
                    [&](tbb::blocked_range<Index> const& r) {
                      for (Index i = r.begin(); i < r.end(); ++i) {
                        const auto& H_i = hessian[i];
                        math::RealMatrix<dim, dim> R = cache[i];
                        math::RealMatrix<dim * dim, dim*(dim + 1)> pfpx = ComputePFPx(R);
                        per_element_hessian[i] = pfpx.transpose() * H_i * pfpx;
                      }
                    });

  for (Index i = 0; i < mesh.GetNumElements(); ++i) {
    const auto& ijk = mesh.GetElement(i);
    const auto& H = per_element_hessian[i];
    for (auto [I, J, Di, Dj] : utils::ndrange<Index>(dim + 1, dim + 1, dim, dim)) {
      Index i_Index = ijk[I];
      Index j_Index = ijk[J];
      Index H_Index_i = I * dim + Di;
      Index H_Index_j = J * dim + Dj;
      Index global_dof_i_Index = i_Index * dim + Di;
      Index global_dof_j_Index = j_Index * dim + Dj;
      coo.push_back({global_dof_i_Index, global_dof_j_Index, H(H_Index_i, H_Index_j)});
    }
  }
  return coo;
}

/*************************
 * SECT: CPU Implementation
 *************************/

template <int dim, template <int> class ElasticModelTemplate>
void ElasticityCompute_CPU<dim, ElasticModelTemplate>::UpdateEnergy() {
  Index const n_elem = this->mesh_->GetNumElements();
  auto const& dg_l = this->deformation_gradient_;
  const bool energy_requires_svd = ElasticModel().EnergyRequiresSvd();
  auto& element_energy = this->energy_on_elements_;
  // tbb::parallel_for(
  //     tbb::blocked_range<Index>(0, n_elem, AX_FEM_COMPUTE_ENERGY_GRAIN),
  //     [&](const tbb::blocked_range<Index>& r) {
  //       ElasticModel model;
  //       math::decomp::SvdResult<dim, Real> tomb;
  //       for (Index i = r.begin(); i < r.end(); ++i) {
  //         auto si = static_cast<size_t>(i);
  //         elasticity::DeformGrad<dim> const& F = dg_l[si];
  //         model.SetLame(this->lame_.col(i));
  //         element_energy[i] = model.Energy(F, energy_requires_svd ? this->svd_results_[si] :
  //         tomb)
  //                             * this->rest_volume_(i);
  //       }
  //     },
  //     this->impl_->e_ap);

  tf::Taskflow flow;
  math::decomp::SvdResult<dim, Real> tomb;
  flow.for_each_index(
      static_cast<Index>(0), n_elem, static_cast<Index>(1),
      [&](Index i) {
        auto si = static_cast<size_t>(i);
        elasticity::DeformGrad<dim> const& F = dg_l[si];
        ElasticModel model;
        model.SetLame(this->lame_.col(i));
        element_energy[i] = model.Energy(F, energy_requires_svd ? this->svd_results_[si] : tomb)
                            * this->rest_volume_(i);
      },
      tf::StaticPartitioner(AX_FEM_COMPUTE_ENERGY_GRAIN));
  this->impl_->executor_.run(flow).wait();
}

template <int dim, template <int> class ElasticModelTemplate>
void ElasticityCompute_CPU<dim, ElasticModelTemplate>::UpdateStress() {
  Index const n_elem = this->mesh_->GetNumElements();
  auto const& dg_l = this->deformation_gradient_;
  auto const& lame = this->lame_;
  const bool stress_requires_svd = ElasticModel().StressRequiresSvd();
  auto& stress = this->stress_on_elements_;
  // tbb::parallel_for(
  //     tbb::blocked_range<Index>(0, n_elem, AX_FEM_COMPUTE_STRESS_GRAIN),
  //     [&](const tbb::blocked_range<Index>& r) {
  //       ElasticModel model;
  //       math::decomp::SvdResult<dim, Real> tomb;
  //       for (Index i = r.begin(); i < r.end(); ++i) {
  //         size_t si = static_cast<size_t>(i);
  //         elasticity::DeformGrad<dim> const& F = dg_l[si];
  //         model.SetLame(lame.col(i));
  //         stress[si] = model.Stress(F, stress_requires_svd ? this->svd_results_[si] : tomb)
  //                      * this->rest_volume_(i);
  //       }
  //     },
  //     this->impl_->s_ap);

  math::decomp::SvdResult<dim, Real> tomb;
  tf::Taskflow flow;
  flow.for_each_index(
      static_cast<Index>(0), n_elem, static_cast<Index>(1),
      [&](Index i) {
        size_t si = static_cast<size_t>(i);
        elasticity::DeformGrad<dim> const& F = dg_l[si];
        ElasticModel model;
        model.SetLame(lame.col(i));
        stress[si] = model.Stress(F, stress_requires_svd ? this->svd_results_[si] : tomb)
                     * this->rest_volume_(i);
      },
      tf::StaticPartitioner(AX_FEM_COMPUTE_STRESS_GRAIN));
  this->impl_->executor_.run(flow).wait();
}

template <int dim, template <int> class ElasticModelTemplate>
void ElasticityCompute_CPU<dim, ElasticModelTemplate>::UpdateHessian(bool projection) {
  Index const n_elem = this->mesh_->GetNumElements();
  auto const& dg_l = this->deformation_gradient_;
  auto const& lame = this->lame_;
  const bool hessian_requires_svd = ElasticModel().HessianRequiresSvd();
  auto& hessian = this->hessian_on_elements_;
  // tbb::parallel_for(
  //     tbb::blocked_range<Index>(0, n_elem, AX_FEM_COMPUTE_HESSIAN_GRAIN),
  //     [&](const tbb::blocked_range<Index>& r) {
  //       ElasticModel model;
  //       math::decomp::SvdResult<dim, Real> tomb;
  //       for (Index i = r.begin(); i < r.end(); ++i) {
  //         const size_t si = static_cast<size_t>(i);
  //         model.SetLame(lame.col(i));
  //         elasticity::DeformGrad<dim> const& F = dg_l.at(si);
  //         hessian.at(si)
  //             = model.Hessian(F, hessian_requires_svd ? (this->svd_results_.at(si)) : tomb)
  //               * this->rest_volume_(i);
  //         if (projection) {
  //           hessian.at(si) = optim::project_spd_by_eigvals<dim * dim>(hessian.at(si), 0.);
  //         }
  //       }
  //     },
  //     this->impl_->h_ap);

  tf::Taskflow flow;
  math::decomp::SvdResult<dim> tomb;
  flow.for_each_index(
      static_cast<Index>(0), n_elem, static_cast<Index>(1),
      [&](Index i) {
        const size_t si = static_cast<size_t>(i);
        ElasticModel model;
        model.SetLame(lame.col(i));
        elasticity::DeformGrad<dim> const& F = dg_l.at(si);
        hessian.at(si) = model.Hessian(F, hessian_requires_svd ? (this->svd_results_.at(si)) : tomb)
                         * this->rest_volume_(i);
        if (projection) {
          hessian.at(si) = optim::project_spd_by_eigvals<dim * dim>(hessian.at(si), 0.);
        }
      },
      tf::StaticPartitioner(AX_FEM_COMPUTE_HESSIAN_GRAIN));
  this->impl_->executor_.run(flow).wait();
}

template <int dim, template <int> class ElasticModelTemplate>
void ElasticityCompute_CPU<dim, ElasticModelTemplate>::GatherEnergyToVertices() {
  auto const& mesh = this->mesh_;
  auto const& e = this->energy_on_elements_;
  auto& result = this->energy_on_vertices_;
  Index n_element = mesh->GetNumElements();
  result.setZero();
  for (Index i = 0; i < n_element; ++i) {
    const auto& ijk = mesh->GetElement(i);
    Real energy = e[i] / Real(dim + 1);
    for (int I = 0; I <= dim; ++I) {
      result(ijk[I]) += energy;
    }
  }
}

template <int dim, template <int> class ElasticModelTemplate>
void ElasticityCompute_CPU<dim, ElasticModelTemplate>::GatherStressToVertices() {
  auto& result = this->stress_on_vertices_;
  result.setZero();
  auto const& mesh = *(this->mesh_);
  auto const& cache = this->rinv_;
  auto const& stress = this->stress_on_elements_;
  // TODO: With Vertex->Element Map, the parallel is possible.
  for (Index i = 0; i < mesh.GetNumElements(); ++i) {
    // For P1 Element, the force on is easy to compute.
    auto const si = static_cast<size_t>(i);
    const auto& ijk = mesh.GetElement(i);
    const auto& stress_i = stress[static_cast<size_t>(si)];
    math::RealMatrix<dim, dim> R = cache[static_cast<size_t>(si)];
    math::RealMatrix<dim, dim> f123 = stress_i * R.transpose();
    for (Index I = 1; I <= dim; ++I) {
      result.col(ijk[I]) += f123.col(I - 1);
      result.col(ijk.x()) -= f123.col(I - 1);
    }
  }
}

template <int dim, template <int> class ElasticModelTemplate>
void ElasticityCompute_CPU<dim, ElasticModelTemplate>::GatherHessianToVertices() {
  math::SparseCOO coo;
  auto& result = this->hessian_on_vertices_;
  auto const& mesh = *(this->mesh_);
  auto const& cache = this->rinv_;
  auto const& hessian = this->hessian_on_elements_;
  size_t total = static_cast<size_t>(mesh.GetNumElements() * dim * dim * (dim + 1) * (dim + 1));
  size_t nE = static_cast<size_t>(mesh.GetNumElements());
  coo.reserve(total);
  std::vector<math::RealMatrix<dim*(dim + 1), dim*(dim + 1)>> per_element_hessian(nE);

  tf::Taskflow flow;
  tf::Executor& executor = this->impl_->executor_;
  tf::Task compute_vert_stress_local_matrix = flow.for_each_index(
      static_cast<size_t>(0), nE, static_cast<size_t>(1),
      [&](size_t si) {
        const auto& H_i = hessian[si];
        math::RealMatrix<dim, dim> R = cache[si];
        math::RealMatrix<dim * dim, dim*(dim + 1)> pfpx = ComputePFPx(R);
        per_element_hessian[si] = pfpx.transpose() * H_i * pfpx;
      },
      tf::StaticPartitioner(AX_FEM_COMPUTE_STRESS_GRAIN));
  tf::Task gather_global = flow.for_each_index(
      static_cast<size_t>(0), this->impl_->outer_start_.size() - 1, static_cast<size_t>(1),
      [&](size_t i) {
        size_t start = this->impl_->outer_start_[i];
        size_t end = this->impl_->outer_start_[i + 1];
        Real* value = result.valuePtr() + i;
        *value = 0;
        for (size_t j = start; j < end; ++j) {
          auto const& entry = this->impl_->hessian_entries_[j];
          Index local_i = entry.local_i_;
          Index local_j = entry.local_j_;
          Index elem_id = entry.element_id_;
          *value += per_element_hessian[elem_id](local_i, local_j);
        }
      },
      tf::StaticPartitioner(AX_FEM_COMPUTE_ENERGY_GRAIN));
  compute_vert_stress_local_matrix.precede(gather_global);
  executor.run(flow).wait();
}

template <int dim, template <int> class ElasticModelTemplate>
void ElasticityCompute_CPU<dim, ElasticModelTemplate>::Update(math::RealField<dim> const& pose,
                                                              ElasticityUpdateLevel upt) {
  Index const n_elem = this->mesh_->GetNumElements();
  auto& dg_l = this->deformation_gradient_;
  auto& svd_results_ = this->svd_results_;
  dg_l.resize(static_cast<size_t>(n_elem));
  svd_results_.resize(static_cast<size_t>(n_elem));
  bool const need_svd
      = (ElasticModel{}.EnergyRequiresSvd() && upt == ElasticityUpdateLevel::Energy)
        || (ElasticModel{}.StressRequiresSvd() && upt == ElasticityUpdateLevel::Stress)
        || (ElasticModel{}.HessianRequiresSvd() && upt == ElasticityUpdateLevel::Hessian);
  // tbb::parallel_for(
  //     tbb::blocked_range<Index>(0, n_elem, AX_FEM_COMPUTE_ENERGY_GRAIN),
  //     [&](const tbb::blocked_range<Index>& r) {
  //       AX_FEM_COMPUTE_SVD_ALGORITHM<dim, Real> svd;
  //       for (Index i = r.begin(); i < r.end(); ++i) {
  //         auto const si = static_cast<size_t>(i);
  //         math::RealMatrix<dim, dim> Ds;
  //         math::IndexVector<dim + 1> element = this->mesh_->GetElement(i);
  //         math::RealVector<dim> local_zero = pose.col(element.x());
  //         for (Index I = 1; I <= dim; ++I) {
  //           Ds.col(I - 1) = pose.col(element[I]) - local_zero;
  //         }
  //         dg_l[si] = Ds * this->rinv_[si];
  //         if (need_svd) {
  //           svd_results_[si] = svd.Solve(dg_l[si]);
  //           math::decomp::svd_remove_rotation(svd_results_[si]);
  //         }
  //       }
  //     },
  //     this->impl_->svd_ap);

  tf::Taskflow flow;
  flow.for_each_index(
      static_cast<Index>(0), n_elem, static_cast<Index>(1),
      [&](Index i) {
        auto const si = static_cast<size_t>(i);
        math::RealMatrix<dim, dim> Ds;
        math::IndexVector<dim + 1> element = this->mesh_->GetElement(i);
        math::RealVector<dim> local_zero = pose.col(element.x());
        for (Index I = 1; I <= dim; ++I) {
          Ds.col(I - 1) = pose.col(element[I]) - local_zero;
        }
        dg_l[si] = Ds * this->rinv_[si];
        if (need_svd) {
          AX_FEM_COMPUTE_SVD_ALGORITHM<dim, Real> svd;
          svd_results_[si] = svd.Solve(dg_l[si]);
          math::decomp::svd_remove_rotation(svd_results_[si]);
        }
      },
      tf::StaticPartitioner(AX_FEM_COMPUTE_ENERGY_GRAIN));
  this->impl_->executor_.run(flow).wait();
}

template <int dim, template <int> class ElasticModelTemplate>
void ElasticityCompute_CPU<dim, ElasticModelTemplate>::RecomputeRestPose() {
  auto const& rest_pose = this->mesh_->GetVertices();
  Index const n_elem = this->mesh_->GetNumElements();
  Index n_vert = this->mesh_->GetNumVertices();
  this->rinv_.reserve(static_cast<size_t>(n_elem));
  Real coef = dim == 3 ? 1.0 / 6.0 : 1.0 / 2.0;
  using namespace math;
  for (Index i = 0; i < n_elem; ++i) {
    RealMatrix<dim, dim> rest_local;
    const auto& element = this->mesh_->GetElement(i);
    const auto& local_zero = rest_pose.col(element.x());
    for (Index I = 1; I <= dim; ++I) {
      rest_local.col(I - 1) = rest_pose.col(element[I]) - local_zero;
    }
    this->rinv_[static_cast<size_t>(i)] = rest_local.inverse();
  }
  // There exist an efficient approximation to compute the volume:
  this->rest_volume_.resize(1, n_elem);
  for (Index i = 0; i < n_elem; ++i) {
    Real v = coef / math::det(this->rinv_[static_cast<size_t>(i)]);
    this->rest_volume_(0, i) = abs(v);
  }

  // 2. prepare the hessian.
  auto& entries = this->impl_->hessian_entries_;
  entries.clear();
  auto elem_accessor = make_accessor(this->mesh_->GetElements());
  for (auto [i, elem] : enumerate(elem_accessor)) {
    for (auto [ei, ej, li, lj] : utils::ndrange<Index>(dim + 1, dim + 1, dim, dim)) {
      Index vert_i = elem[ei], vert_j = elem[ej];
      Index dof_i = vert_i * dim + li, dof_j = vert_j * dim + lj;
      Index local_i = ei * dim + li, local_j = ej * dim + lj;
      entries.push_back(HessianEntryInfo{i, local_i, local_j, dof_i, dof_j});
    }
  }
  std::sort(entries.begin(), entries.end(),
            [&](HessianEntryInfo const& a, HessianEntryInfo const& b) {
              return prec(n_elem * dim, a, b);
            });
  // 2.2 make a demo, and let eigen fill in the entries:
  SparseCOO coo;
  for (const auto& entry : entries) {
    coo.push_back(SparseEntry(entry.global_i_, entry.global_j_, 1));
  }
  this->hessian_on_vertices_ = make_sparse_matrix(dim * n_vert, dim * n_vert, coo);
  this->hessian_on_vertices_.makeCompressed();
  // 2.3 examine that the cache is correct.
  std::vector<size_t>& outer_start = impl_->outer_start_;
  outer_start.push_back(0);
  math::for_each_entry(this->hessian_on_vertices_, [&](Index i, Index j, Real) {
    size_t last_end = outer_start.back();
    // The first entry should be the same as the last end.
    AX_DCHECK(entries[last_end].global_i_ == i && entries[last_end].global_j_ == j,
              "(Internal) Hessian Entry not match.");
    while (last_end < entries.size()) {
      // find all the i, j entry
      last_end += 1;
      if (entries[last_end].global_i_ != i || entries[last_end].global_j_ != j) {
        break;
      }
    }
    outer_start.push_back(last_end);
  });

  // { // NOTE: Tested.
  //   size_t ii = 0;
  //   for_each_entry(this->hessian_on_vertices_, [&](Index i, Index j, Real v) {
  //     auto start = outer_start[ii];
  //     auto end = outer_start[ii + 1];
  //     // auto gi = entries[start].global_i_, gj = entries[start].global_j_;
  //     // AX_INFO("SparseMatrix({}, {})={}, HessianGatter[{}->{}]=({}, {})",
  //     //         i, j, static_cast<size_t>(v),
  //     //         start, end, gi, gj);
  //     AX_CHECK(end - start == static_cast<size_t>(v), "Hessian Entry size not match.");
  //     for (size_t v: range(start, end)) {
  //       auto gi = entries[v].global_i_, gj = entries[v].global_j_;
  //       AX_CHECK(gi == i && gj == j, "Hessian Entry not match.");
  //     }
  //     ++ii;
  //   });
  // }

  // prepare all the buffers.
  ElasticityComputeBase<dim>::RecomputeRestPose();
}

template <int dim, template <int> class ElasticModelTemplate>
ElasticityCompute_CPU<dim, ElasticModelTemplate>::ElasticityCompute_CPU(
    std::shared_ptr<TriMesh<dim>> mesh)
    : ElasticityComputeBase<dim>(mesh) {
  this->impl_ = std::make_unique<Impl>();
}

template <int dim, template <int> class ElasticModelTemplate>
ElasticityCompute_CPU<dim, ElasticModelTemplate>::~ElasticityCompute_CPU() {}

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
