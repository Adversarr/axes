//
// Created by JerryYang on 2024/3/24.
//
#include "axes/pde/fem/deform.hpp"

#include <tbb/parallel_for.h>

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
template <idx dim> static DeformationGradientCache<dim> dg_rpcache_p1(
    MeshBase<dim> const& mesh, typename MeshBase<dim>::vertex_list_t const& rest_pose) {
  AX_DCHECK(mesh.GetNumVerticesPerElement() == dim + 1)
      << "P1 Element means dim+1 = #v per element.";
  DeformationGradientCache<dim> cache;
  idx n_elem = mesh.GetNumElements();
  cache.resize(n_elem);

  tbb::parallel_for<idx>(0, n_elem, [&](idx i) {
    matr<dim, dim> rest_local;
    const auto& element = mesh.GetElement(i);
    const auto& local_zero = rest_pose.col(element.x());
    for (idx I = 1; I <= dim; ++I) {
      rest_local.col(I - 1) = rest_pose.col(element[I]) - local_zero;
    }
    cache[i] = rest_local.inverse();
  });

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
  tbb::parallel_for<idx>(0, n_elem, [&](idx i) {
    matr<dim, dim> curr_local;
    const auto& element = mesh.GetElement(i);
    const auto& local_zero = mesh.GetVertex(element.x());
    for (idx I = 1; I <= dim; ++I) {
      curr_local.col(I - 1) = mesh.GetVertex(element[I]) - local_zero;
    }
    dg[i] = curr_local * cache[i];
  });

  return dg;
}

template <idx dim>
typename MeshBase<dim>::vertex_list_t dg_tsv_p1(MeshBase<dim> const& mesh,
                                                List<elasticity::StressTensor<dim>> const& stress,
                                                DeformationGradientCache<dim> const & cache) {
  typename MeshBase<dim>::vertex_list_t result;
  result.setZero(dim, mesh.GetNumVertices());
  for (idx i = 0; i < mesh.GetNumElements(); ++i) {
    // For P1 Element, the force on is easy to compute.
    const auto& ijk = mesh.GetElement(i);
    const auto& stress_i = stress[i];
    math::matr<dim, dim> R = cache[i];
    // TODO: Is this correct?
    math::matr<dim, dim> stress_i_R = stress_i * R;
    for (idx I = 0; I < dim; ++I) {
      auto force = stress_i.col(I);
      result.col(ijk[I + 1]) += force;
      result.col(ijk.x()) -= force;
    }
  }
  return result;
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

template <idx dim> typename MeshBase<dim>::vertex_list_t Deformation<dim>::StressToForce(
    List<elasticity::StressTensor<dim>> const& stress) const {
  idx n_element = mesh_.GetNumElements();
  { // Check input size:
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
  AX_CHECK(false) << "Not Implemented Error";
  AX_UNREACHABLE();
}

template class Deformation<2>;
template class Deformation<3>;

}  // namespace ax::pde::fem