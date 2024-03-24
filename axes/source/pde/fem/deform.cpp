//
// Created by JerryYang on 2024/3/24.
//
#include <tbb/parallel_for.h>

#include "axes/pde/fem/deform.hpp"

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
  return has_error;
}

template <idx dim>
static DeformationGradientCache<dim> dg_rpcache_p1(MeshBase<dim> const& mesh) {
  AX_DCHECK(mesh.GetNumVerticesPerElement() == dim + 1)
      << "P1 Element means dim+1 = #v per element.";
  DeformationGradientCache<dim> cache;
  idx n_elem = mesh.GetNumElements();
  cache.resize(n_elem);

  tbb::parallel_for<idx>(0, n_elem, [&cache, &mesh](idx i) {
    matr<dim, dim> rest_local;
    const auto& element = mesh.GetElement(i);
    const auto& local_zero = mesh.GetVertex(element.x());
#pragma unroll
    for (idx I = 1; I <= dim; ++I) {
      rest_local.col(i) = mesh.GetVertex(element[I]) - local_zero;
    }
    cache[i] = rest_local.inverse();
  });

#ifndef NDEBUG
  if (!check_cache<dim>(mesh)) {
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
#pragma unroll
    for (idx I = 1; I <= dim; ++I) {
      curr_local.col(i) = mesh.GetVertex(element[I]) - local_zero;
    }
    dg[i] = curr_local * cache[i];
  });

  return dg;
}

/***********************************************************************************************
 * Interfaces
 ***********************************************************************************************/

template <idx dim> DeformationGradientList<dim> compute_deformation_gradient(
    MeshBase<dim> const& mesh, typename MeshBase<dim>::vertex_list_t const& rest_pose) {
  auto cache = compute_deformation_gradient_rest_pose_cache<dim>(rest_pose);
  return compute_deformation_gradient_cached(mesh, cache);
}

template <idx dim> DeformationGradientCache<dim> compute_deformation_gradient_rest_pose_cache(
    MeshBase<dim> const& mesh) {
  if (mesh.GetType() == MeshType::kP1) {
    return dg_rpcache_p1(mesh);
  } else {
    AX_CHECK(false) << "Not Implemented";
  }
  AX_UNREACHABLE();
}

template <idx dim> DeformationGradientList<dim> compute_deformation_gradient_cached(
    MeshBase<dim> const& mesh, DeformationGradientCache<dim> const& cache) {
  if (mesh.GetType() == MeshType::kP1) {
    return dg_p1(mesh, cache);
  } else {
    AX_CHECK(false) << "Not Implemented";
  }
  AX_UNREACHABLE();
}

template DeformationGradientList<3> compute_deformation_gradient(
    MeshBase<3> const& mesh, typename MeshBase<3>::vertex_list_t const&);
template DeformationGradientList<2> compute_deformation_gradient(
    MeshBase<2> const& mesh, typename MeshBase<2>::vertex_list_t const&);

template DeformationGradientCache<2> compute_deformation_gradient_rest_pose_cache(
    MeshBase<2> const&);
template DeformationGradientCache<3> compute_deformation_gradient_rest_pose_cache(
    MeshBase<3> const&);

}  // namespace ax::pde::elasticity