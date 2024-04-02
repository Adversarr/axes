//
// Created by JerryYang on 2024/3/24.
//
#include "axes/pde/fem/deform.hpp"
#include "axes/utils/iota.hpp"
#include "axes/utils/time.hpp"

#include "axes/utils/parallel_for_helper.hpp"

// #define DOUBLE_CHECK

namespace ax::pde::fem {
using namespace elasticity;
using namespace math;
template <idx dim> static bool check_cache(DeformationGradientCache<dim> const& cache) {
  bool has_error = false;
  for (const auto& R_inv : cache) {
    real detR = R_inv.determinant();
    if (math::isnan(detR)) {
      AX_LOG(ERROR) << "Found Determinant of R.inv is nan.";
      has_error = true;
    }
  }
  return !has_error;
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

AX_FORCE_INLINE static math::mat2r ApplyPFPx(math::mat2r const& P,
  math::mat2r const& R) {
  /*{{Subscript[P, 1, 1] Subscript[r, 1, 1] +
   Subscript[P, 1, 2] Subscript[r, 1, 2],
  Subscript[P, 1, 1] Subscript[r, 2, 1] +
   Subscript[P, 1, 2] Subscript[r, 2, 2]}, {Subscript[P, 2, 1]
     Subscript[r, 1, 1] + Subscript[P, 2, 2] Subscript[r, 1, 2],
  Subscript[P, 2, 1] Subscript[r, 2, 1] +
   Subscript[P, 2, 2] Subscript[r, 2, 2]}}*/

  math::mat2r result;
  result(0, 0) = P(0, 0) * R(0, 0) + P(0, 1) * R(0, 1);
  result(0, 1) = P(0, 0) * R(1, 0) + P(0, 1) * R(1, 1);
  result(1, 0) = P(1, 0) * R(0, 0) + P(1, 1) * R(0, 1);
  result(1, 1) = P(1, 0) * R(1, 0) + P(1, 1) * R(1, 1);
  return result;
}
AX_FORCE_INLINE static math::matr<3, 3> ApplyPFPx(math::mat3r const& P,
  math::mat3r const& R) {
  /*{{Subscript[P, 1,1] Subscript[r, 1,1]+Subscript[P, 1,2] Subscript[r, 1,2]+Subscript[P, 1,3]
   * Subscript[r, 1,3],Subscript[P, 1,1] Subscript[r, 2,1]+Subscript[P, 1,2] Subscript[r,
   * 2,2]+Subscript[P, 1,3] Subscript[r, 2,3],Subscript[P, 1,1] Subscript[r, 3,1]+Subscript[P, 1,2]
   * Subscript[r, 3,2]+Subscript[P, 1,3] Subscript[r, 3,3]},{Subscript[P, 2,1] Subscript[r,
   * 1,1]+Subscript[P, 2,2] Subscript[r, 1,2]+Subscript[P, 2,3] Subscript[r, 1,3],Subscript[P, 2,1]
   * Subscript[r, 2,1]+Subscript[P, 2,2] Subscript[r, 2,2]+Subscript[P, 2,3] Subscript[r,
   * 2,3],Subscript[P, 2,1] Subscript[r, 3,1]+Subscript[P, 2,2] Subscript[r, 3,2]+Subscript[P, 2,3]
   * Subscript[r, 3,3]},{Subscript[P, 3,1] Subscript[r, 1,1]+Subscript[P, 3,2] Subscript[r,
   * 1,2]+Subscript[P, 3,3] Subscript[r, 1,3],Subscript[P, 3,1] Subscript[r, 2,1]+Subscript[P, 3,2]
   * Subscript[r, 2,2]+Subscript[P, 3,3] Subscript[r, 2,3],Subscript[P, 3,1] Subscript[r,
   * 3,1]+Subscript[P, 3,2] Subscript[r, 3,2]+Subscript[P, 3,3] Subscript[r, 3,3]}}*/
  math::mat3r result;
  result(0, 0) = P(0, 0) * R(0, 0) + P(0, 1) * R(0, 1) + P(0, 2) * R(0, 2);
  result(0, 1) = P(0, 0) * R(1, 0) + P(0, 1) * R(1, 1) + P(0, 2) * R(1, 2);
  result(0, 2) = P(0, 0) * R(2, 0) + P(0, 1) * R(2, 1) + P(0, 2) * R(2, 2);
  result(1, 0) = P(1, 0) * R(0, 0) + P(1, 1) * R(0, 1) + P(1, 2) * R(0, 2);
  result(1, 1) = P(1, 0) * R(1, 0) + P(1, 1) * R(1, 1) + P(1, 2) * R(1, 2);
  result(1, 2) = P(1, 0) * R(2, 0) + P(1, 1) * R(2, 1) + P(1, 2) * R(2, 2);
  result(2, 0) = P(2, 0) * R(0, 0) + P(2, 1) * R(0, 1) + P(2, 2) * R(0, 2);
  result(2, 1) = P(2, 0) * R(1, 0) + P(2, 1) * R(1, 1) + P(2, 2) * R(1, 2);
  result(2, 2) = P(2, 0) * R(2, 0) + P(2, 1) * R(2, 1) + P(2, 2) * R(2, 2);
  return result;
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

template <idx dim> static DeformationGradientCache<dim> dg_rpcache_p1(
    MeshBase<dim> const& mesh, typename MeshBase<dim>::vertex_list_t const& rest_pose) {
  AX_DCHECK(mesh.GetNumVerticesPerElement() == dim + 1)
      << "P1 Element means dim+1 = #v per element.";
  DeformationGradientCache<dim> cache;
  idx n_elem = mesh.GetNumElements();
  cache.resize(n_elem);

  AX_PARALLEL_FOR_BEGIN(0, n_elem, i)
    matr<dim, dim> rest_local;
    const auto& element = mesh.GetElement(i);
    const auto& local_zero = rest_pose.col(element.x());
    for (idx I = 1; I <= dim; ++I) {
      rest_local.col(I - 1) = rest_pose.col(element[I]) - local_zero;
    }
    cache[i] = rest_local.inverse();
  AX_PARALLEL_FOR_END();

#ifndef NDEBUG
  if (!check_cache<dim>(cache)) {
    AX_LOG(ERROR) << "Mesh Cache computation failed! Please check the input mesh.";
  }
#endif
  return cache;
}

template <idx dim>
static DeformationGradientList<dim> dg_p1(MeshBase<dim> const& mesh,
                                          DeformationGradientCache<dim> const& cache) {
  idx n_elem = mesh.GetNumElements();
  DeformationGradientList<dim> dg(n_elem);
  AX_PARALLEL_FOR_BEGIN(0, n_elem, i)
    matr<dim, dim> curr_local;
    const auto& element = mesh.GetElement(i);
    const auto& local_zero = mesh.GetVertex(element.x());
    for (idx I = 1; I <= dim; ++I) {
      curr_local.col(I - 1) = mesh.GetVertex(element[I]) - local_zero;
    }
    dg[i] = curr_local * cache[i];
  AX_PARALLEL_FOR_END();

  return dg;
}

template <idx dim>
typename MeshBase<dim>::vertex_list_t dg_tsv_p1(MeshBase<dim> const& mesh,
                                                List<elasticity::StressTensor<dim>> const& stress,
                                                DeformationGradientCache<dim> const& cache) {
  typename MeshBase<dim>::vertex_list_t result;
  result.setZero(dim, mesh.GetNumVertices());
  // TODO: With Vertex->Element Map, the parallel is possible.
  for (idx i = 0; i < mesh.GetNumElements(); ++i) {
    // For P1 Element, the force on is easy to compute.
    const auto& ijk = mesh.GetElement(i);
    const auto& stress_i = stress[i];
    math::matr<dim, dim> R = cache[i];
    // TODO: Fine, but the efficiency is not good.
    math::matr<dim, dim> F = ApplyPFPx(stress_i, R);
    #ifdef DOUBLE_CHECK
    math::matr<dim * dim, dim * (dim + 1)> pfpx = ComputePFPx(R);
    math::vecr<dim * (dim + 1)> F2 = pfpx.transpose() * math::flatten(stress_i);
    for (idx I = 0; I <= dim; ++I) {
      result.col(ijk[I]) += F2.template segment<dim>(I * dim);
    }
    std::cout << math::norm(F2.template segment<dim>(dim) - F.col(0)) << std::endl;
    std::cout << math::norm(F2.template segment<dim>(2 * dim) - F.col(1)) << std::endl;
    #endif
    for (idx I = 1; I <= dim; ++I) {
      result.col(ijk[I]) += F.col(I-1);
      result.col(ijk[0]) -= F.col(I-1);
    }
  }
  return result;
}

template <idx dim>
math::sp_coeff_list dg_thv_p1(MeshBase<dim> const& mesh,
                                                List<elasticity::HessianTensor<dim>> const& hessian,
                                                DeformationGradientCache<dim> const& cache) {
  math::sp_coeff_list coo;
  coo.reserve(mesh.GetNumElements() * dim * dim * (dim + 1) * (dim + 1));
  std::vector<math::matr<dim * (dim + 1), dim * (dim + 1)>> per_element_hessian(mesh.GetNumElements());
  tbb::parallel_for(tbb::blocked_range<idx>(0, mesh.GetNumElements(), dim * dim * dim * 5), 
    [&](tbb::blocked_range<idx> const& r) {
      for (idx i = r.begin(); i < r.end(); ++i) {
        const auto& H_i = hessian[i];
        math::matr<dim, dim> R = cache[i];
        math::matr<dim * dim, dim * (dim + 1)> pfpx = ComputePFPx(R);
        per_element_hessian[i] = pfpx.transpose() * H_i * pfpx;
      }
  });

  for (idx i = 0; i < mesh.GetNumElements(); ++i) {
    const auto &ijk = mesh.GetElement(i);
    const auto &H = per_element_hessian[i];
    for (auto [I, J, Di, Dj]: utils::multi_iota(dim+1, dim+1, dim, dim)) {
      idx i_idx = ijk[I];
      idx j_idx = ijk[J];
      idx H_idx_i = I * dim + Di;
      idx H_idx_j = J * dim + Dj;
      coo.push_back(math::sp_coeff{i_idx, j_idx, H(H_idx_i, H_idx_j)});
    }
  }
  return coo;
}

/***********************************************************************************************
 * Interfaces
 ***********************************************************************************************/

template <idx dim>
Deformation<dim>::Deformation(MeshBase<dim> const& mesh,
                              typename MeshBase<dim>::vertex_list_t const& rest_pose)
    : mesh_(mesh) {
  this->UpdateRestPose(rest_pose);
}

template <idx dim>
void Deformation<dim>::UpdateRestPose(typename MeshBase<dim>::vertex_list_t const& rest_pose) {
  rest_pose_ = rest_pose;
  if (mesh_.GetType() == MeshType::kP1) {
    deformation_gradient_cache_ = dg_rpcache_p1<dim>(mesh_, rest_pose);
  } else {
    AX_CHECK(false) << "Not Implemented Error";
  }
  // There exist an efficient approximation to compute the volume:
  rest_pose_volume_.resize(1, mesh_.GetNumElements());
  for (idx i = 0; i < mesh_.GetNumElements(); ++i) {
    rest_pose_volume_(0, i) = 1.0 / math::abs(math::det(deformation_gradient_cache_[i]));
  }
}

template <idx dim> DeformationGradientList<dim> Deformation<dim>::Forward() const {
  if (mesh_.GetType() == MeshType::kP1) {
    return dg_p1<dim>(mesh_, deformation_gradient_cache_);
  } else {
    AX_CHECK(false) << "Not Implemented";
  }
  AX_UNREACHABLE();
}

template <idx dim>
elasticity::DeformationGradientCache<dim> const& Deformation<dim>::GetRestPoseCache() const {
  return deformation_gradient_cache_;
}

template <idx dim> math::field1r dg_tev_p1(MeshBase<dim> const& mesh_, math::field1r const& e,
                                           field1r const& rest_pose_volume_) {
  idx n_element = mesh_.GetNumElements();
  math::field1r result(1, mesh_.GetNumVertices());
  result.setZero();
  for (idx i = 0; i < n_element; ++i) {
    const auto& ijk = mesh_.GetElement(i);
    real energy = e[i] * rest_pose_volume_(0, i) / real(dim + 1);
    for (idx I = 0; I < dim; ++I) {
      result(ijk[I + 1]) += energy;
      result(ijk.x()) += energy;
    }
  }
  return result;
}

template <idx dim> math::field1r Deformation<dim>::EnergyToVertices(math::field1r const& e) const {
  idx n_element = mesh_.GetNumElements();
  {  // Check input size:
    size_t e_size = e.size();
    AX_CHECK_EQ(e_size, n_element) << "#energy != #element";
  }
  if (mesh_.GetType() == MeshType::kP1) {
    return dg_tev_p1<dim>(mesh_, e, rest_pose_volume_);
  } else {
    AX_CHECK(false) << "Not Implemented Error";
  }
  AX_UNREACHABLE();
}

template <idx dim> typename MeshBase<dim>::vertex_list_t Deformation<dim>::StressToVertices(
    List<elasticity::StressTensor<dim>> const& stress) const {
  idx n_element = mesh_.GetNumElements();
  {  // Check input size:
    size_t stress_size = stress.size();
    AX_CHECK_EQ(stress_size, n_element) << "#stress != #element";
  }
  if (mesh_.GetType() == MeshType::kP1) {
    return dg_tsv_p1<dim>(mesh_, stress, deformation_gradient_cache_);
  } else {
    AX_CHECK(false) << "Not Implemented Error";
  }
  AX_UNREACHABLE();
}

template <idx dim> math::sp_coeff_list Deformation<dim>::HessianToVertices(
    List<elasticity::HessianTensor<dim>> const& hessian) const {
  if (mesh_.GetType() == MeshType::kP1) {
    return dg_thv_p1<dim>(mesh_, hessian, deformation_gradient_cache_);
  } else {
    AX_CHECK(false) << "Not Implemented Error";
  }
  AX_UNREACHABLE();
}

template class Deformation<2>;
template class Deformation<3>;

}  // namespace ax::pde::fem