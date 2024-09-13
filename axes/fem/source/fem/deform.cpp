//
// Created by JerryYang on 2024/3/24.
//
#include "ax/fem/deform.hpp"

#include <fmt/ostream.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "ax/math/utils/formatting.hpp"
#include "ax/math/traits.hpp"
#include "ax/utils/ndrange.hpp"

namespace ax::fem {
using namespace elasticity;
using namespace math;
template <int dim> static bool check_cache(DeformGradCache<dim> const& cache) {
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

template <int dim> static DeformGradCache<dim> dg_rpcache_p1(
    LinearMesh<dim> const& mesh, typename LinearMesh<dim>::vertex_list_t const& rest_pose) {
  DeformGradCache<dim> cache;
  Index n_elem = mesh.GetNumElements();
  cache.resize(static_cast<size_t>(n_elem));

  for (Index i = 0; i < n_elem; ++i) {
    RealMatrix<dim, dim> rest_local;
    const auto& element = mesh.GetElement(i);
    const auto& local_zero = rest_pose.col(element.x());
    for (Index I = 1; I <= dim; ++I) {
      rest_local.col(I - 1) = rest_pose.col(element[I]) - local_zero;
    }
    cache[static_cast<size_t>(i)] = rest_local.inverse();
  }
  if (!check_cache<dim>(cache)) {
    AX_ERROR("Mesh Cache computation failed! Please check the input mesh.");
  }
  return cache;
}

template <int dim>
static DeformGradBuffer<dim> dg_p1(LinearMesh<dim> const& mesh, RealField<dim> const& pose,
                                          DeformGradCache<dim> const& Dm_inv) {
  Index n_elem = mesh.GetNumElements();
  DeformGradBuffer<dim> dg(static_cast<size_t>(n_elem));
  static tbb::affinity_partitioner ap;
  tbb::parallel_for(
      tbb::blocked_range<Index>(0, n_elem, 50000 / (dim * dim)),
      [&](tbb::blocked_range<Index> const& r) {
        for (Index i = r.begin(); i < r.end(); ++i) {
          size_t const si = static_cast<size_t>(i);
          math::RealMatrix<dim, dim> Ds;
          math::IndexVector<dim + 1> element = mesh.GetElement(i);
          math::RealVector<dim> local_zero = pose.col(element.x());
          for (Index I = 1; I <= dim; ++I) {
            Ds.col(I - 1) = pose.col(element[I]) - local_zero;
          }
          dg[si] = Ds * Dm_inv[si];
        }
      },
      ap);

  return dg;
}

template <int dim> typename LinearMesh<dim>::vertex_list_t dg_tsv_p1(
    LinearMesh<dim> const& mesh, std::vector<elasticity::StressTensor<dim>> const& stress,
    DeformGradCache<dim> const& cache) {
  typename LinearMesh<dim>::vertex_list_t result;
  result.setZero(dim, mesh.GetNumVertices());
  // TODO: With Vertex->Element Map, the parallel is possible.
  for (Index i = 0; i < mesh.GetNumElements(); ++i) {
    // For P1 Element, the force on is easy to compute.
    const auto& ijk = mesh.GetElement(i);
    const auto& stress_i = stress[static_cast<size_t>(i)];
    math::RealMatrix<dim, dim> R = cache[static_cast<size_t>(i)];
    math::RealMatrix<dim, dim> f123 = stress_i * R.transpose();
    for (Index I = 1; I <= dim; ++I) {
      result.col(ijk[I]) += f123.col(I - 1);
      result.col(ijk.x()) -= f123.col(I - 1);
    }
  }
  return result;
}

template <int dim>
math::SparseCOO dg_thv_p1(LinearMesh<dim> const& mesh,
                              std::vector<elasticity::HessianTensor<dim>> const& hessian,
                              DeformGradCache<dim> const& cache) {
  math::SparseCOO coo;
  size_t total = static_cast<size_t>(mesh.GetNumElements() * dim * dim * (dim + 1) * (dim + 1));
  size_t nE = static_cast<size_t>(mesh.GetNumElements());
  coo.reserve(total);
  std::vector<math::RealMatrix<dim*(dim + 1), dim*(dim + 1)>> per_element_hessian(nE);
  tbb::parallel_for(tbb::blocked_range<Index>(0, mesh.GetNumElements(), 500000 / dim * dim * dim),
                    [&](tbb::blocked_range<Index> const& r) {
                      for (Index i = r.begin(); i < r.end(); ++i) {
                        size_t si = static_cast<size_t>(i);
                        const auto& H_i = hessian[si];
                        math::RealMatrix<dim, dim> R = cache[si];
                        math::RealMatrix<dim * dim, dim*(dim + 1)> pfpx = ComputePFPx(R);
                        per_element_hessian[si] = pfpx.transpose() * H_i * pfpx;
                      }
                    });

  for (Index i = 0; i < mesh.GetNumElements(); ++i) {
    const auto& ijk = mesh.GetElement(i);
    const auto& H = per_element_hessian[static_cast<size_t>(i)];
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

/***********************************************************************************************
 * Interfaces
 ***********************************************************************************************/

template <int dim>
Deformation<dim>::Deformation(LinearMesh<dim> const& mesh,
                              typename LinearMesh<dim>::vertex_list_t const& rest_pose)
    : mesh_(mesh) {
  this->UpdateRestPose(rest_pose);
}

template <int dim>
void Deformation<dim>::UpdateRestPose(typename LinearMesh<dim>::vertex_list_t const& rest_pose) {
  rest_pose_ = rest_pose;
  deformation_gradient_cache_ = dg_rpcache_p1<dim>(mesh_, rest_pose);
  // There exist an efficient approximation to compute the volume:
  Real coef = dim == 3 ? 1.0 / 6.0 : 1.0 / 2.0;
  rest_pose_volume_.resize(1, mesh_.GetNumElements());
  for (Index i = 0; i < mesh_.GetNumElements(); ++i) {
    Real v = coef / math::det(deformation_gradient_cache_[static_cast<size_t>(i)]);
    if (v < 0) {
      AX_WARN("Negative volume detected: {}: {}", i, mesh_.GetElement(i).transpose());
    }
    rest_pose_volume_(0, i) = abs(v);
  }
}

template <int dim> DeformGradBuffer<dim> Deformation<dim>::Forward() const {
  return Forward(mesh_.GetVertices());
}

template <int dim> DeformGradBuffer<dim> Deformation<dim>::Forward(
    typename LinearMesh<dim>::vertex_list_t const& current) const {
  return dg_p1<dim>(mesh_, current, deformation_gradient_cache_);
}

template <int dim>
elasticity::DeformGradCache<dim> const& Deformation<dim>::GetRestPoseCache() const {
  return deformation_gradient_cache_;
}

template <int dim> math::RealField1 dg_tev_p1(LinearMesh<dim> const& mesh_, math::RealField1 const& e) {
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

template <int dim> math::RealField1 Deformation<dim>::EnergyToVertices(math::RealField1 const& e) const {
  Index n_element = mesh_.GetNumElements();
  Index e_size = e.size();
  AX_CHECK(e_size == n_element, "#energy != #element");
  return dg_tev_p1<dim>(mesh_, e);
}

template <int dim> typename LinearMesh<dim>::vertex_list_t Deformation<dim>::StressToVertices(
    std::vector<elasticity::StressTensor<dim>> const& stress) const {
  Index n_element = mesh_.GetNumElements();
  Index stress_size = static_cast<size_t>(stress.size());
  AX_DCHECK(stress_size == n_element, "#stress != #element");
  return dg_tsv_p1<dim>(mesh_, stress, deformation_gradient_cache_);
}

template <int dim> math::SparseCOO Deformation<dim>::HessianToVertices(
    std::vector<elasticity::HessianTensor<dim>> const& hessian) const {
  return dg_thv_p1<dim>(mesh_, hessian, deformation_gradient_cache_);
}

template class Deformation<2>;
template class Deformation<3>;

}  // namespace ax::fem
